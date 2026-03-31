#!/usr/bin/env python3
"""让 Baxter 展示安全舞蹈（头部、双臂、夹爪、屏幕）。"""

import os
import time

import cv2
import cv_bridge
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image as ImageMsg


LEFT_HOME = {
    'left_s0': -0.133,
    'left_s1': 0.388,
    'left_e0': -0.538,
    'left_e1': 0.949,
    'left_w0': -0.422,
    'left_w1': -0.026,
    'left_w2': -0.276,
}

RIGHT_HOME = {
    'right_s0': -0.133,
    'right_s1': 0.388,
    'right_e0': 0.338,
    'right_e1': 0.949,
    'right_w0': -0.422,
    'right_w1': -0.026,
    'right_w2': -0.276,
}

RIGHT_WAVE = {
    'right_s0': -0.50,
    'right_s1': -0.15,
    'right_e0': 1.5,
    'right_e1': 1.4,
    'right_w0': -0.8,
    'right_w1': -1.0,
    'right_w2': -0.3,
}

LEFT_WAVE = {
    'left_s0': 0.5,
    'left_s1': -0.20,
    'left_e0': -1.4,
    'left_e1': 1.2,
    'left_w0': 0.0,
    'left_w1': -0.5,
    'left_w2': 0.3,
}


def send_image(path):
    if not os.path.isfile(path):
        rospy.logwarn('dance: image not found %s', path)
        return

    img = cv2.imread(path)
    if img is None:
        rospy.logwarn('dance: failed to load %s', path)
        return

    img = cv2.resize(img, (1024, 600))
    bridge = cv_bridge.CvBridge()
    msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    if not hasattr(send_image, '_pub'):
        send_image._pub = rospy.Publisher('/robot/xdisplay', ImageMsg, latch=True, queue_size=1)
        rospy.sleep(0.1)
    send_image._pub.publish(msg)
    rospy.sleep(2.0)


class BaxterDance:
    def __init__(self):
        self.enable_robot()
        self.left_limb = baxter_interface.Limb('left')
        self.right_limb = baxter_interface.Limb('right')
        self.head = baxter_interface.Head()
        self.gripper = baxter_interface.Gripper('right', CHECK_VERSION)
        self.left_limb.set_joint_position_speed(0.3)
        self.right_limb.set_joint_position_speed(0.3)
        if self.gripper.error():
            self.gripper.reset()
            rospy.sleep(0.5)
        if not self.gripper.calibrated() and self.gripper.type() != 'custom':
            rospy.loginfo('dance: calibrating right gripper')
            self.gripper.calibrate()
            rospy.sleep(0.6)

    def enable_robot(self):
        enabler = baxter_interface.RobotEnable(CHECK_VERSION)
        if not enabler.state().enabled:
            rospy.loginfo('dance: enabling robot for choreography')
            enabler.enable()
            rospy.sleep(1.0)

    def move_to_pose(self, limb, pose):
        limb.move_to_joint_positions(pose, timeout=10.0, threshold=0.01)

    def head_nod(self):
        self.head.set_pan(0.5)
        rospy.sleep(0.5)
        self.head.command_nod()
        rospy.sleep(0.5)
        self.head.set_pan(-0.4)
        rospy.sleep(0.5)
        self.head.command_nod()
        rospy.sleep(0.5)
        self.head.set_pan(0.0)
        rospy.sleep(0.5)

    def dance(self, image_paths):
        rospy.loginfo('dance: moving arms to home positions')
        self.move_to_pose(self.left_limb, LEFT_HOME)
        self.move_to_pose(self.right_limb, RIGHT_HOME)
        rospy.sleep(0.5)

        send_image(image_paths[0])

        rospy.loginfo('dance: warming up head')
        self.head_nod()

        rospy.loginfo('dance: right arm waving')
        for _ in range(3):
            self.move_to_pose(self.right_limb, RIGHT_WAVE)
            self.move_to_pose(self.left_limb, LEFT_WAVE)
            rospy.sleep(0.5)
            self.move_to_pose(self.right_limb, RIGHT_HOME)
            self.move_to_pose(self.left_limb, LEFT_HOME)
            rospy.sleep(0.5)

        rospy.loginfo('dance: playful gripper hiccup')
        self.gripper.open()
        rospy.sleep(0.5)
        self.gripper.close()
        rospy.sleep(0.5)
        self.gripper.open()

        send_image(image_paths[1])

        rospy.loginfo('dance: stepping to final pose safely')
        self.move_to_pose(self.left_limb, LEFT_HOME)
        self.move_to_pose(self.right_limb, RIGHT_HOME)
        self.head.set_pan(0.0)


if __name__ == '__main__':
    rospy.init_node('dance_node')
    paths = rospy.get_param('~image_paths', ['/root/baxter_ws/src/greet_demo/pic/hello.jpg',
                                            '/root/baxter_ws/src/greet_demo/pic/goodbye.jpg'])
    dancer = BaxterDance()
    dancer.dance(paths)