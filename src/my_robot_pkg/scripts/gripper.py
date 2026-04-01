#!/usr/bin/env python3
import threading

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import String

import baxter_interface
from baxter_interface import CHECK_VERSION

class VisionTriggeredGrasp:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_topic = rospy.get_param('~image_topic', '/cameras/right_hand_camera/image')
		self.use_range_trigger = rospy.get_param('~use_range_trigger', True)
		self.range_topic = rospy.get_param('~range_topic', '/robot/range/right_hand_range/state')
		self.range_trigger_m = rospy.get_param('~range_trigger_m', 0.08)
		self.enable_center_trigger = rospy.get_param('~enable_center_trigger', True)
		self.center_roi_ratio = rospy.get_param('~center_roi_ratio', 0.35)
		self.min_area = rospy.get_param('~min_area', 2500)
		self.stable_frames_required = rospy.get_param('~stable_frames_required', 3)
		self.trigger_cooldown = rospy.get_param('~trigger_cooldown', 4.0)
		self.debug_view = rospy.get_param('~debug_view', )

		self._stable_count = 0
		self._last_trigger_time = rospy.Time(0)
		self._busy = False
		self._awaiting_release = False
		self._busy_lock = threading.Lock()

		self._init_robot()
		rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=1)
		if self.use_range_trigger:
			rospy.Subscriber(self.range_topic, Range, self._on_range, queue_size=1)
		self.release_topic = rospy.get_param('~release_topic', '/gripper/release')
		rospy.Subscriber(self.release_topic, String, self._on_release_cmd, queue_size=1)
		rospy.loginfo(
			'gripper node started, image_topic=%s range_topic=%s release_topic=%s',
			self.image_topic, self.range_topic, self.release_topic)

	def _init_robot(self):
		enabler = baxter_interface.RobotEnable(CHECK_VERSION)
		if not enabler.state().enabled:
			rospy.loginfo('enabling robot...')
			enabler.enable()
			rospy.sleep(1.0)

		self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
		if self.right_gripper.error():
			self.right_gripper.reset()
			rospy.sleep(0.5)
		if (not self.right_gripper.calibrated()) and self.right_gripper.type() != 'custom':
			rospy.loginfo('calibrating right gripper...')
			self.right_gripper.calibrate()
			rospy.sleep(0.5)

	def _on_image(self, msg):
		if self._busy or self._awaiting_release:
			return

		detected, area, in_center = self._detect_object(msg)
		if detected:
			if self.enable_center_trigger and in_center and self._cooldown_ready():
				self._stable_count = 0
				self._try_start_grasp(area, source='vision-center', close_only=True)
				return

			self._stable_count += 1
			if self._stable_count >= self.stable_frames_required and self._cooldown_ready():
				self._stable_count = 0
				self._try_start_grasp(area, source='vision-stable', close_only=False)
		else:
			self._stable_count = 0

	def _on_range(self, msg):
		if self._busy or self._awaiting_release or (not self._cooldown_ready()):
			return

		r = float(msg.range)
		if msg.min_range < r < self.range_trigger_m:
			self._stable_count = 0
			self._try_start_grasp(int(r * 1000), source='range', close_only=True)

	def _on_release_cmd(self, msg):
		if msg.data.strip().lower() != 'q':
			return
		if not self._awaiting_release:
			return
		rospy.loginfo('release command received (%s), opening gripper', msg.data)
		self.right_gripper.open()
		rospy.sleep(0.5)
		self._busy = False
		self._awaiting_release = False

	def _cooldown_ready(self):
		elapsed = (rospy.Time.now() - self._last_trigger_time).to_sec()
		return elapsed >= self.trigger_cooldown

	def _try_start_grasp(self, area, source='vision', close_only=False):
		with self._busy_lock:
			if self._busy:
				return
			self._busy = True
		self._last_trigger_time = rospy.Time.now()
		rospy.loginfo('trigger=%s value=%d close_only=%s, start grasp sequence', source, area, str(close_only))
		thread = threading.Thread(target=self._grasp_sequence, args=(close_only,))
		thread.daemon = True
		thread.start()

	def _detect_object(self, msg):
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		except Exception as exc:
			rospy.logwarn_throttle(2.0, 'cv_bridge error: %s', str(exc))
			return False, 0, False

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# Detect green target in HSV
		lower_green = (40, 90, 60)
		upper_green = (85, 255, 255)
		mask = cv2.inRange(hsv, lower_green, upper_green)

		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
		mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		largest_area = 0
		largest_contour = None
		for cnt in contours:
			area = cv2.contourArea(cnt)
			if area > largest_area:
				largest_area = int(area)
				largest_contour = cnt

		in_center = False
		if largest_contour is not None and largest_area >= self.min_area:
			moments = cv2.moments(largest_contour)
			if moments['m00'] > 0:
				cx = int(moments['m10'] / moments['m00'])
				cy = int(moments['m01'] / moments['m00'])
				h, w = mask.shape
				hw = int((w * self.center_roi_ratio) * 0.5)
				hh = int((h * self.center_roi_ratio) * 0.5)
				x0 = (w // 2) - hw
				x1 = (w // 2) + hw
				y0 = (h // 2) - hh
				y1 = (h // 2) + hh
				in_center = (x0 <= cx <= x1) and (y0 <= cy <= y1)

				if self.debug_view:
					cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 0), 2)
					cv2.circle(frame, (cx, cy), 6, (0, 255, 0), -1)

		if self.debug_view:
			cv2.imshow('gripper_mask', mask)
			cv2.imshow('gripper_frame', frame)
			cv2.waitKey(1)

		return largest_area >= self.min_area, largest_area, in_center

	def _grasp_sequence(self, close_only=False):
		try:
			if close_only:
				rospy.loginfo('close-only trigger, closing right gripper')
			else:
				rospy.loginfo('vision trigger, closing right gripper')
			self.right_gripper.close()
			rospy.sleep(0.7)

			self._awaiting_release = True
			rospy.loginfo('grasp complete, waiting for q release')
			return
		except Exception as exc:
			rospy.logerr('grasp sequence failed: %s', str(exc))
			self._busy = False
			self._awaiting_release = False


if __name__ == '__main__':
	rospy.init_node('gripper_node')
	VisionTriggeredGrasp()
	rospy.spin()
