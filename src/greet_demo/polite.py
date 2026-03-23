#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import os
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

LEFT_JOINTS = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
RIGHT_JOINTS = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

def safe_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default

def maybe_set_gripper(gripper, value):
    pos = safe_float(value, None)
    if pos is None:
        return
    pos = max(0.0, min(100.0, pos))
    try:
        gripper.command_position(pos)
    except Exception:
        pass

def load_rows(path):
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        rows = [row for row in reader]
    if not rows:
        raise RuntimeError("动作文件为空: {}".format(path))
    return rows

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", required=True, help="动作文件 .dat 路径")
    parser.add_argument("--speed", type=float, default=1.0, help="回放速度倍率，>1 更快")
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        raise RuntimeError("找不到文件: {}".format(args.file))
    if args.speed <= 0:
        raise RuntimeError("--speed 必须 > 0")

    rospy.init_node("polite_player", anonymous=True)

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    if not rs.state().enabled:
        rs.enable()

    left_limb = baxter_interface.Limb("left")
    right_limb = baxter_interface.Limb("right")

    left_gripper = baxter_interface.Gripper("left", CHECK_VERSION)
    right_gripper = baxter_interface.Gripper("right", CHECK_VERSION)
    try:
        left_gripper.calibrate()
    except Exception:
        pass
    try:
        right_gripper.calibrate()
    except Exception:
        pass

    rows = load_rows(args.file)
    prev_t = None

    for row in rows:
        if rospy.is_shutdown():
            break

        now_t = safe_float(row.get("time", 0.0), 0.0)
        if prev_t is not None:
            dt = max(0.0, (now_t - prev_t) / args.speed)
            if dt > 0:
                rospy.sleep(dt)
        prev_t = now_t

        left_cmd = {j: safe_float(row.get(j, 0.0), 0.0) for j in LEFT_JOINTS}
        right_cmd = {j: safe_float(row.get(j, 0.0), 0.0) for j in RIGHT_JOINTS}

        left_limb.set_joint_positions(left_cmd)
        right_limb.set_joint_positions(right_cmd)

        maybe_set_gripper(left_gripper, row.get("left_gripper", "100"))
        maybe_set_gripper(right_gripper, row.get("right_gripper", "100"))

    rospy.loginfo("polite 回放完成: %s", args.file)

if __name__ == "__main__":
    main()