#!/usr/bin/env python3
import threading
import time

import moveit_commander
import rospy
from std_msgs.msg import String


class PresetExecutor:
    def __init__(self):
        moveit_commander.roscpp_initialize([])

        self.semantic_param = rospy.get_param('~semantic_param', '/robot_description_semantic')
        self.semantic_timeout = rospy.get_param('~semantic_timeout', 10.0)
        self.move_group_wait_for_servers = rospy.get_param('~move_group_wait_for_servers', 20.0)
        self.left_group_name = rospy.get_param('~left_group', 'left_arm')
        self.right_group_name = rospy.get_param('~right_group', 'right_arm')
        self.both_group_name = rospy.get_param('~both_group', '')

        self.trigger_cooldown = rospy.get_param('~trigger_cooldown', 1.2)
        self.estop_lock_sec = rospy.get_param('~estop_lock_sec', 2.0)
        self.wave_cycles = rospy.get_param('~wave_cycles', 3)
        self.poses = rospy.get_param('~poses', {})

        self.enabled = False
        self.estop_until = 0.0
        self.last_trigger = {}
        self.lock = threading.Lock()
        self._init_lock = threading.Lock()

        self.robot = None
        self.left = None
        self.right = None
        self.both = None

        self._init_moveit(wait_timeout=self.semantic_timeout)
        self._retry_timer = rospy.Timer(rospy.Duration(1.0), self._on_retry_timer)

        rospy.Subscriber('/gesture/event', String, self._on_event, queue_size=10)
        rospy.loginfo('preset_executor ready. enabled=%s', self.enabled)

    @staticmethod
    def _set_safe_scaling(group):
        group.set_max_velocity_scaling_factor(0.25)
        group.set_max_acceleration_scaling_factor(0.2)

    def _wait_for_semantic_description(self, timeout=None):
        if rospy.has_param(self.semantic_param):
            return
        rospy.logwarn('waiting for semantic description: %s', self.semantic_param)
        timeout_sec = float(self.semantic_timeout if timeout is None else timeout)
        deadline = time.time() + timeout_sec
        rate = rospy.Rate(5)
        while (not rospy.is_shutdown()) and (time.time() < deadline):
            if rospy.has_param(self.semantic_param):
                return
            rate.sleep()
        rospy.logwarn('semantic description still missing after %.1fs: %s',
                      timeout_sec, self.semantic_param)

    def _has_usable_group(self):
        return self.left is not None or self.right is not None or self.both is not None

    def _init_moveit(self, wait_timeout=0.0):
        with self._init_lock:
            if self.robot is not None and self._has_usable_group():
                return True

            if wait_timeout and wait_timeout > 0.0:
                self._wait_for_semantic_description(timeout=wait_timeout)
            elif not rospy.has_param(self.semantic_param):
                rospy.logwarn_throttle(5.0, 'semantic description unavailable: %s', self.semantic_param)
                return False

            self.robot = self._make_robot_commander()
            available_groups = self.robot.get_group_names() if self.robot else []
            rospy.loginfo('available planning groups: %s', ', '.join(available_groups) if available_groups else '<none>')

            self.left_group_name = self._resolve_group_name(
                requested=self.left_group_name,
                candidates=['left_arm', 'left_arm_torso', 'left_manipulator'],
                available=available_groups,
            )
            self.right_group_name = self._resolve_group_name(
                requested=self.right_group_name,
                candidates=['right_arm', 'right_arm_torso', 'right_manipulator'],
                available=available_groups,
            )
            self.both_group_name = self._resolve_group_name(
                requested=self.both_group_name,
                candidates=['both_arms', 'arms', 'dual_arms'],
                available=available_groups,
            )

            self.left = self._make_group(self.left_group_name)
            self.right = self._make_group(self.right_group_name)
            self.both = self._make_group(self.both_group_name) if self.both_group_name else None

            if self._has_usable_group():
                rospy.loginfo('moveit groups initialized successfully')
                return True

            rospy.logwarn_throttle(5.0, 'no usable MoveIt group yet, will retry')
            return False

    def _on_retry_timer(self, _event):
        if self._has_usable_group():
            return
        self._init_moveit(wait_timeout=0.0)

    @staticmethod
    def _make_robot_commander():
        try:
            return moveit_commander.RobotCommander()
        except Exception as exc:
            rospy.logwarn('failed RobotCommander init: %s', str(exc))
            return None

    @staticmethod
    def _resolve_group_name(requested, candidates, available):
        if not available:
            return requested
        if requested and requested in available:
            return requested
        for candidate in candidates:
            if candidate in available:
                rospy.logwarn("group '%s' unavailable, fallback to '%s'", requested, candidate)
                return candidate
        rospy.logwarn("group '%s' unavailable and no fallback match in [%s]",
                      requested, ', '.join(candidates))
        return ''

    def _make_group(self, group_name):
        if not group_name:
            return None
        try:
            group = moveit_commander.MoveGroupCommander(
                group_name,
                wait_for_servers=float(self.move_group_wait_for_servers),
            )
            self._set_safe_scaling(group)
            return group
        except Exception as exc:
            rospy.logwarn('failed group [%s]: %s', group_name, str(exc))
            return None

    def _cooldown_ok(self, key):
        now = time.time()
        last = self.last_trigger.get(key, 0.0)
        if now - last < self.trigger_cooldown:
            return False
        self.last_trigger[key] = now
        return True

    def _stop_all(self):
        for group in [self.left, self.right, self.both]:
            if group is None:
                continue
            try:
                group.stop()
                group.clear_pose_targets()
            except Exception:
                pass

    @staticmethod
    def _merge_current_with_partial(group, partial):
        active_joints = group.get_active_joints()
        current_values = group.get_current_joint_values()
        merged = dict(zip(active_joints, current_values))
        for name, value in partial.items():
            if name in merged:
                merged[name] = float(value)
        return merged

    def _go_partial(self, group, partial_target):
        if group is None or not partial_target:
            return False
        joint_target = self._merge_current_with_partial(group, partial_target)
        group.set_joint_value_target(joint_target)
        ok = group.go(wait=True)
        group.stop()
        return bool(ok)

    def _home_both(self):
        home_left = self.poses.get('home_left', {})
        home_right = self.poses.get('home_right', {})

        if self.both is not None:
            merged = {}
            merged.update(home_left)
            merged.update(home_right)
            return self._go_partial(self.both, merged)

        left_ok = self._go_partial(self.left, home_left) if self.left else True
        right_ok = self._go_partial(self.right, home_right) if self.right else True
        return left_ok and right_ok

    def _wave_right(self):
        if self.right is None:
            rospy.logerr('right arm group unavailable')
            return False

        ready = self.poses.get('wave_ready_right', {})
        pose_1 = self.poses.get('wave_right_1', {})
        pose_2 = self.poses.get('wave_right_2', {})

        if not self._go_partial(self.right, ready):
            return False

        for _ in range(self.wave_cycles):
            if not self._go_partial(self.right, pose_1):
                return False
            if not self._go_partial(self.right, pose_2):
                return False

        return self._go_partial(self.right, ready)

    def _on_event(self, msg):
        label = msg.data.strip().upper()
        now = time.time()

        if not self._has_usable_group() and not self._init_moveit(wait_timeout=0.0):
            return

        with self.lock:
            if label == 'OPEN_PALM':
                self._stop_all()
                self.enabled = False
                self.estop_until = now + self.estop_lock_sec
                rospy.logwarn('ESTOP: disabled for %.1fs', self.estop_lock_sec)
                return

            if now < self.estop_until:
                return

            if label == 'FIST' and self._cooldown_ok('FIST'):
                self.enabled = not self.enabled
                rospy.loginfo('enabled => %s', self.enabled)
                return

            if not self.enabled:
                return

            if not self._cooldown_ok(label):
                return

            if label == 'V_SIGN':
                self._home_both()
            elif label == 'WAVE':
                self._wave_right()


if __name__ == '__main__':
    rospy.init_node('preset_executor')
    PresetExecutor()
    rospy.spin()
