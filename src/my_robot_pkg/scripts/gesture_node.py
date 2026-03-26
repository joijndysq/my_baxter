#!/usr/bin/env python3
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    import cv2
    import mediapipe as mp
    HAS_VISION = True
except Exception:
    HAS_VISION = False


class GestureNode:
    def __init__(self):
        self.pub_event = rospy.Publisher('/gesture/event', String, queue_size=10)
        self.pub_label = rospy.Publisher('/gesture/label', String, queue_size=10)

        self.min_interval = rospy.get_param('~min_interval', 1.0)
        self.use_mock = rospy.get_param('~use_mock', False)

        self._last_emit = {}
        self.bridge = CvBridge()

        if self.use_mock or not HAS_VISION:
            rospy.Subscriber('/gesture/mock', String, self._on_mock, queue_size=10)
            rospy.logwarn('gesture_node running in MOCK mode (or mediapipe/cv2 missing).')
            return

        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
        rospy.Subscriber('/camera/rgb/image_raw', Image, self._on_image, queue_size=1)

    def _emit(self, label):
        now = time.time()
        last = self._last_emit.get(label, 0.0)
        if now - last < self.min_interval:
            return
        self._last_emit[label] = now
        self.pub_label.publish(String(label))
        self.pub_event.publish(String(label))
        rospy.loginfo('gesture event: %s', label)

    def _on_mock(self, msg):
        label = msg.data.strip().upper()
        if label:
            self._emit(label)

    @staticmethod
    def _is_extended(lm_tip, lm_pip):
        return lm_tip.y < lm_pip.y

    def _classify(self, hand_landmarks):
        lm = hand_landmarks.landmark

        thumb = lm[4].x > lm[3].x
        index = self._is_extended(lm[8], lm[6])
        middle = self._is_extended(lm[12], lm[10])
        ring = self._is_extended(lm[16], lm[14])
        pinky = self._is_extended(lm[20], lm[18])

        cnt = sum([thumb, index, middle, ring, pinky])

        if cnt <= 1:
            return 'FIST'
        if cnt >= 4:
            return 'OPEN_PALM'
        if index and middle and (not ring) and (not pinky):
            return 'V_SIGN'
        if thumb and index and middle and (not ring) and (not pinky):
            return 'WAVE'
        return None

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.mp_hands.process(rgb)
            if not result.multi_hand_landmarks:
                return
            label = self._classify(result.multi_hand_landmarks[0])
            if label:
                self._emit(label)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'gesture_node image callback error: %s', str(exc))


if __name__ == '__main__':
    rospy.init_node('gesture_node')
    GestureNode()
    rospy.spin()
