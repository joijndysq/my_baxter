#!/usr/bin/env python3

import rospy

import math
import tf
import baxter_interface
import os
import sys
import time
from moveit_commander import conversions
from geometry_msgs.msg import ( PoseStamped,
                                Pose,
                                Point,
                                Quaternion )
from std_msgs.msg import Header
# import std_srvs.srv
from baxter_core_msgs.srv import ( SolvePositionIK,
                                   SolvePositionIKRequest )

# golf setup class
class golf_setup():
    "Golf setup program"
    def __init__(self, arm):
        # initialise ros node
        rospy.init_node("Locate", anonymous = True)

        # arm ("left" or "right")
        self.limb           = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)
        self.gripper.calibrate()

        # image directory
        self.image_dir = os.getenv("HOME") + "/Visual_Servoing/"

        # start positions (use safer defaults for Noetic/sim IK)
        if self.limb == "left":
            self.pose_candidates = [
                [0.60, 0.30, 0.20, -1.0 * math.pi, 0.0, 0.0],
                [0.55, 0.40, 0.20, -1.0 * math.pi, 0.0, 0.0],
                [0.65, 0.25, 0.18, -1.0 * math.pi, 0.0, 0.0],
            ]
        else:
            self.pose_candidates = [
                [0.60, -0.30, 0.20, -1.0 * math.pi, 0.0, 0.0],
                [0.55, -0.40, 0.20, -1.0 * math.pi, 0.0, 0.0],
                [0.65, -0.25, 0.18, -1.0 * math.pi, 0.0, 0.0],
            ]

        self.pose = self.pose_candidates[0]

        # distance to table
        self.distance = 0.0

        # Enable the actuators
        baxter_interface.RobotEnable().enable()

        # set speed as a ratio of maximum speed
        self.limb_interface.set_joint_position_speed(0.5)

        # move to first valid start position
        moved = False
        for candidate_pose in self.pose_candidates:
            if self.baxter_ik_move(self.limb, candidate_pose):
                moved = True
                break

        if not moved:
            rospy.logwarn("No valid IK start pose found; using current endpoint pose as startup pose.")
            self.update_pose_from_endpoint()

    # move a limb
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException) as error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            return False
        if (ik_response.isValid[0]):
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            self.limb_interface.move_to_joint_positions(limb_joints)
        else:
            rospy.logwarn("IK invalid for pose: %s", rpy_pose)
            return False

        position, euler = self.update_pose_from_endpoint()

        print("             request   actual")
        print('front back = %5.4f ' % rpy_pose[0], "%5.4f" % position[0])
        print('left right = %5.4f ' % rpy_pose[1], "%5.4f" % position[1])
        print('up down    = %5.4f ' % rpy_pose[2], "%5.4f" % position[2])
        print('roll       = %5.4f ' % rpy_pose[3], "%5.4f" % euler[0])
        print('pitch      = %5.4f ' % rpy_pose[4], "%5.4f" % euler[1])
        print('yaw        = %5.4f ' % rpy_pose[5], "%5.4f" % euler[2])

        # remember actual position achieved
        return True

    def update_pose_from_endpoint(self):
        endpoint_pose = self.limb_interface.endpoint_pose()
        position_obj = endpoint_pose['position']
        orientation_obj = endpoint_pose['orientation']

        position = [position_obj.x, position_obj.y, position_obj.z]
        euler = tf.transformations.euler_from_quaternion(
            [orientation_obj.x, orientation_obj.y, orientation_obj.z, orientation_obj.w]
        )

        self.pose = [position[0], position[1], position[2], euler[0], euler[1], euler[2]]
        return position, euler

    # find distance of limb from nearest object
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        if dist > 65000:
            sys.exit("ERROR - get_distance - no distance found")

        return float(dist / 1000.0)

    # save setup values
    def save(self):
        os.makedirs(self.image_dir, exist_ok=True)
        setup_file = os.path.join(self.image_dir, "setup.dat")
        with open(setup_file, "w") as setup_handle:
            s = 'limb = %s\n' % self.limb
            setup_handle.write(s)
            s = 'distance = %s\n' % self.distance
            setup_handle.write(s)

def main():
    limb  = "left"
    setup = golf_setup(limb)

    # open the gripper
    setup.gripper.open()

    # find distance of arm to table
    setup.distance = setup.get_distance(setup.limb)
    print("distance = ", setup.distance)

    setup.save()

if __name__ == "__main__":
    main()

# rospy.spin()

