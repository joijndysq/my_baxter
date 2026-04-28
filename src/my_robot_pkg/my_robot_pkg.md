# my_robot_pkg 技术报告

## 1 项目概述

my_robot_pkg 是基于 ROS 的 Baxter 机器人自定义功能包，包含多个实用脚本，用于控制 Baxter 机器人的手臂动作、夹爪抓取和视觉伺服等功能。

### 1.1 目录结构

```
my_robot_pkg/
├── package.xml           # 包配置文件
├── CMakeLists.txt       # 编译配置
├── scripts/             # 可执行脚本
│   ├── dance.py         # 舞蹈动作表演
│   ├── gripper.py     # 视觉伺服抓取
│   └── end.py          # 双臂协调动作序列
├── src/                # C++源码目录
├── include/            # 头文件目录
├── gui/                # GUI资源
└── model/              # 模型文件 (best.pt - PyTorch模型)
```

---

## 2 dance.py 舞蹈动作脚本

### 2.1 功能概述

dance.py 实现了一个简单的舞蹈表演程序，包含以下动作：
- 手臂归位到自然下垂位置
- 头部点头动作
- 双臂挥手动作
- 夹爪开合动作
- 头部显示屏显示图片

### 2.2 核心代码

```python
#!/usr/bin/env python3
import os
import time

import cv2
import cv_bridge
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image as ImageMsg


# 左臂自然下垂位置
LEFT_HOME = {
    'left_s0': -0.133,
    'left_s1': 0.388,
    'left_e0': -0.538,
    'left_e1': 0.949,
    'left_w0': -0.422,
    'left_w1': -0.026,
    'left_w2': -0.276,
}

# 右臂自然下垂位置
RIGHT_HOME = {
    'right_s0': -0.133,
    'right_s1': 0.388,
    'right_e0': 0.338,
    'right_e1': 0.949,
    'right_w0': -0.422,
    'right_w1': -0.026,
    'right_w2': -0.276,
}

# 右臂挥手位置
RIGHT_WAVE = {
    'right_s0': -0.50,
    'right_s1': -0.15,
    'right_e0': 1.5,
    'right_e1': 1.4,
    'right_w0': -0.8,
    'right_w1': -1.0,
    'right_w2': -0.3,
}

# 左臂挥手位置
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
    """发送图片到Baxter头部显示屏"""
    if not os.path.isfile(path):
        rospy.logwarn('dance: image not found %s', path)
        return

    img = cv2.imread(path)
    if img is None:
        rospy.logwarn('dance: failed to load %s', path)
        return

    # 缩放到Baxter显示屏分辨率 1024x600
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
        
        # 设置关节运动速度
        self.left_limb.set_joint_position_speed(0.3)
        self.right_limb.set_joint_position_speed(0.3)
        
        # 校准夹爪
        if self.gripper.error():
            self.gripper.reset()
            rospy.sleep(0.5)
        if not self.gripper.calibrated() and self.gripper.type() != 'custom':
            rospy.loginfo('dance: calibrating right gripper')
            self.gripper.calibrate()
            rospy.sleep(0.6)

    def enable_robot(self):
        """启用机��人"""
        enabler = baxter_interface.RobotEnable(CHECK_VERSION)
        if not enabler.state().enabled:
            rospy.loginfo('dance: enabling robot for choreography')
            enabler.enable()
            rospy.sleep(1.0)

    def move_to_pose(self, limb, pose):
        """移动到指定姿态"""
        limb.move_to_joint_positions(pose, timeout=10.0, threshold=0.01)

    def head_nod(self):
        """头部点头动作"""
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
        """执行完整舞蹈序列"""
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
```

### 2.3 动作流程

```
1. 初始化
   ├── 启用机器人
   ├── 获取左右臂 Limb 对象
   ├── 获取头部 Head 对象
   ├── 获取右夹爪 Gripper 对象
   └── 校准夹爪

2. 舞蹈动作
   ├── 手臂归位 (LEFT_HOME, RIGHT_HOME)
   ├── 显示第一张图片
   ├── 头部点头 (左右摆动 + command_nod)
   ├── 挥手动作 3次循环
   │   ├── RIGHT_WAVE + LEFT_WAVE
   │   └── RIGHT_HOME + LEFT_HOME
   ├── 夹爪开合动作
   └── 显示第二张图片
```

---

## 3 gripper.py 视觉伺服抓取脚本

### 3.1 功能概述

gripper.py 实现了基于视觉的自动抓取功能：

- 订阅右手相机图像话题
- 使用 OpenCV 进行绿色物体检测
- 基于距离传感器的触发
- 自动控制夹爪闭合抓取物体
- 支持释放命令

### 3.2 核心代码

```python
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
        
        # 图像话题配置
        self.image_topic = rospy.get_param('~image_topic', '/cameras/right_hand_camera/image')
        self.use_range_trigger = rospy.get_param('~use_range_trigger', True)
        self.range_topic = rospy.get_param('~range_topic', '/robot/range/right_hand_range/state')
        
        # 触发参数配置
        self.range_trigger_m = rospy.get_param('~range_trigger_m', 0.08)
        self.enable_center_trigger = rospy.get_param('~enable_center_trigger', True)
        self.center_roi_ratio = rospy.get_param('~center_roi_ratio', 0.35)
        self.min_area = rospy.get_param('~min_area', 2500)
        self.stable_frames_required = rospy.get_param('~stable_frames_required', 3)
        self.trigger_cooldown = rospy.get_param('~trigger_cooldown', 4.0)
        self.debug_view = rospy.get_param('~debug_view', )

        # 状态变量
        self._stable_count = 0
        self._last_trigger_time = rospy.Time(0)
        self._busy = False
        self._awaiting_release = False
        self._busy_lock = threading.Lock()

        # 初始化机器人
        self._init_robot()
        
        # 订阅话题
        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=1)
        if self.use_range_trigger:
            rospy.Subscriber(self.range_topic, Range, self._on_range, queue_size=1)
        
        self.release_topic = rospy.get_param('~release_topic', '/gripper/release')
        rospy.Subscriber(self.release_topic, String, self._on_release_cmd, queue_size=1)
        
        rospy.loginfo(
            'gripper node started, image_topic=%s range_topic=%s release_topic=%s',
            self.image_topic, self.range_topic, self.release_topic)

    def _init_robot(self):
        """初始化机器人"""
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
        """图像回调函数"""
        if self._busy or self._awaiting_release:
            return

        detected, area, in_center = self._detect_object(msg)
        if detected:
            # 中心触发：检测到物体在画面中心
            if self.enable_center_trigger and in_center and self._cooldown_ready():
                self._stable_count = 0
                self._try_start_grasp(area, source='vision-center', close_only=True)
                return

            # 稳定触发：连续多帧检测到物体
            self._stable_count += 1
            if self._stable_count >= self.stable_frames_required and self._cooldown_ready():
                self._stable_count = 0
                self._try_start_grasp(area, source='vision-stable', close_only=False)
        else:
            self._stable_count = 0

    def _on_range(self, msg):
        """距离传感器回调"""
        if self._busy or self._awaiting_release or (not self._cooldown_ready()):
            return

        r = float(msg.range)
        if msg.min_range < r < self.range_trigger_m:
            self._stable_count = 0
            self._try_start_grasp(int(r * 1000), source='range', close_only=True)

    def _on_release_cmd(self, msg):
        """释放命令回调"""
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
        """检查冷却时间是否就绪"""
        elapsed = (rospy.Time_now() - self._last_trigger_time).to_sec()
        return elapsed >= self.trigger_cooldown

    def _try_start_grasp(self, area, source='vision', close_only=False):
        """尝试开始抓取"""
        with self._busy_lock:
            if self._busy:
                return
            self._busy = True
        self._last_trigger_time = rospy.Time.now()
        rospy.loginfo('trigger=%s value=%d close_only=%s, start grasp sequence', source, area, str(close_only))
        
        # 在新线程中执行抓取序列
        thread = threading.Thread(target=self._grasp_sequence, args=(close_only,))
        thread.daemon = True
        thread.start()

    def _detect_object(self, msg):
        """检测绿色物体"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'cv_bridge error: %s', str(exc))
            return False, 0, False

        # 转换为HSV色彩空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 绿色HSV范围
        lower_green = (40, 90, 60)
        upper_green = (85, 255, 255)
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_area = 0
        largest_contour = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > largest_area:
                largest_area = int(area)
                largest_contour = cnt

        # 检查是否在中心区域
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
        """抓取序列"""
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
```

### 3.3 工作原理

#### 3.3.1 视觉检测流程

```
相机图像 (ROS Image 消息)
        │
        ▼
┌─────────────────┐
│ cv_bridge       │  ◄── 转换为 OpenCV 格式
│ 格式转换       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ HSV 色彩空间    │  ◄── BGR → HSV
│ 转换           │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ inRange 滤色   │  ◄── 绿色范围 (40,90,60) ~ (85,255,255)
│ 绿色掩膜       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 形态学操作     │  ◄── 开运算 + 闭运算
│ 去噪          │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ findContours   │  ◄── 查找最大轮廓
│ 轮廓检测       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ 面积计算       │  ◄── cv2.contourArea
│ 和中心检测     │
└─────────────────┘
```

#### 3.3.2 触发条件

| 触发类型 | 条件 | 说明 |
|---------|------|------|
| 中心触发 | 绿色物体在画面中心且面积>2500 | 仅闭合夹爪 |
| 稳定触发 | 连续3帧检测到绿色物体且面积>2500 | 完全抓取 |
| 距离触发 | 距离传感器<8cm | 仅闭合夹爪 |

### 3.4 参数配置

| 参数 | 默认值 | 说明 |
|------|-------|------|
| ~image_topic | /cameras/right_hand_camera/image | 相机图像话题 |
| ~range_topic | /robot/range/right_hand_range/state | 距离传感器话题 |
| ~use_range_trigger | True | 是否启用距离触发 |
| ~range_trigger_m | 0.08 | 距离触发阈值(米) |
| ~enable_center_trigger | True | 是否启用中心触发 |
| ~center_roi_ratio | 0.35 | 中心区域比例 |
| ~min_area | 2500 | 最小面积阈值 |
| ~stable_frames_required | 3 | 稳定帧数要求 |
| ~trigger_cooldown | 4.0 | 触发冷却时间(秒) |
| ~debug_view | False | 调试视图开关 |
| ~release_topic | /gripper/release | 释放命令话题 |

---

## 4 end.py 双臂协调动作序列脚本

### 4.1 功能概述

end.py 实现了一个复杂的双臂协调动作序列，包含：

- 多个预定义关节姿态
- 平滑插值移动算法
- 图片显示功能
- 完整的动作序列编排

### 4.2 核心代码

```python
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import os
import time
from std_msgs.msg import String
import baxter_interface
from baxter_interface import Limb

# ==================== 关节角配置 ====================
# 左臂初始/归零位置（自然下垂）
LEFT_INITIAL_JOINT_ANGLES = {
    'left_s0': -0.133,
    'left_s1': 0.388,
    'left_e0': -0.538,
    'left_e1': 0.949,
    'left_w0': -0.422,
    'left_w1': -0.026,
    'left_w2': -0.276
}

# 右臂初始/归零位置（自然下垂）
RIGHT_INITIAL_JOINT_ANGLES = {
    'right_s0': -0.133,
    'right_s1': 0.388,
    'right_e0': 0.338,
    'right_e1': 0.949,
    'right_w0': -0.422,
    'right_w1': -0.026,
    'right_w2': -0.276
}

# 右臂打招呼动作位置
RIGHT_HELLO_JOINT_ANGLES = {    
    'right_s0': -0.459,    # 肩部外展
    'right_s1': -0.202,    # 肩部前屈
    'right_e0': 1.807,     # 肘部内旋
    'right_e1': 1.714,     # 肘部伸展
    'right_w0': -0.906,    # 腕部保持中立
    'right_w1': -1.545,    # 腕部伸展
    'right_w2': -0.276     # 腕部旋转初始
}

RIGHT_HELLO_JOINT_END_ANGLES = {
    'right_s0': -0.395,
    'right_s1': -0.202,
    'right_e0': 1.831,
    'right_e1': 1.981,
    'right_w0': -1.979,
    'right_w1': -1.100,
    'right_w2': -0.448
}

# 右臂平举上抬位置
RIGHT_RAISE_JOINT_ANGLES = {
    'right_s0': -0.097,
    'right_s1': 0.118,
    'right_e0': 0.393,
    'right_e1': -0.692,
    'right_w0': 0.206,
    'right_w1': 0.055,
    'right_w2': -0.161
}

# 右臂模仿拍球动作 - 准备姿态
RIGHT_DRIBBLE_PREP_JOINT_ANGLES = {
    'right_s0': -0.240,
    'right_s1': -0.200,
    'right_e0': 0.720,
    'right_e1': 1.300,
    'right_w0': -0.180,
    'right_w1': -0.700,
    'right_w2': -0.060
}

# 右臂模仿拍球动作 - 上抬位
RIGHT_DRIBBLE_UP_JOINT_ANGLES = {
    'right_s0': -0.200,
    'right_s1': -0.080,
    'right_e0': 0.640,
    'right_e1': 1.120,
    'right_w0': -0.100,
    'right_w1': -0.520,
    'right_w2': 0.000
}

# 右臂模仿拍球动作 - 下压位
RIGHT_DRIBBLE_DOWN_JOINT_ANGLES = {
    'right_s0': -0.280,
    'right_s1': -0.320,
    'right_e0': 0.820,
    'right_e1': 1.520,
    'right_w0': -0.260,
    'right_w1': -0.900,
    'right_w2': -0.120
}

# 运动控制参数
MOTION_PARAMS = {
    'move_timeout': 15.0,
    'position_tolerance': 0.00872665,
    'joint_velocity_limit': 0.3
}


def initialize_left_arm():
    """初始化左臂到自然下垂位置"""
    try:
        rospy.loginfo("初始化左臂到自然下垂位置...")
        left_limb = baxter_interface.Limb('left')
        
        if not baxter_interface.RobotEnable().state().enabled:
            baxter_interface.RobotEnable().enable()
            rospy.sleep(1.0)
        
        left_limb.set_joint_position_speed(MOTION_PARAMS['joint_velocity_limit'])
        move_to_joint_positions_smooth(left_limb, LEFT_INITIAL_JOINT_ANGLES)
        
        rospy.loginfo("左臂初始化完成 - 自然下垂位置")
        return left_limb
        
    except Exception as e:
        rospy.logerr(f"左臂初始化失败: {e}")
        raise


def initialize_right_arm():
    """初始化右臂到自然下垂位置"""
    try:
        rospy.loginfo("初始化右臂到自然下垂位置...")
        right_limb = baxter_interface.Limb('right')
        
        right_limb.set_joint_position_speed(MOTION_PARAMS['joint_velocity_limit'])
        move_to_joint_positions_smooth(right_limb, RIGHT_INITIAL_JOINT_ANGLES)
        
        rospy.loginfo("右臂初始化完成 - 自然下垂位置")
        return right_limb
        
    except Exception as e:
        rospy.logerr(f"右臂初始化失败: {e}")
        raise


def move_to_joint_positions_smooth(limb, target_angles, timeout=15.0, 
                                   max_step=0.1, interpolation_steps=30):
    """
    平滑移动到目标关节位置，采用分段插值实现平滑运动
    """
    try:
        # 获取当前关节角度
        current_angles = limb.joint_angles()
        
        # 验证关节名称
        joint_names = limb.joint_names()
        for joint_name in target_angles.keys():
            if joint_name not in joint_names:
                rospy.logerr(f"关节名称错误: {joint_name}")
                return False
        
        # 计算关节角度变化
        angle_differences = {}
        for joint in target_angles:
            if joint in current_angles:
                diff = target_angles[joint] - current_angles[joint]
                angle_differences[joint] = diff
        
        # 分段插值
        for step in range(1, interpolation_steps + 1):
            intermediate_angles = {}
            for joint in target_angles:
                if joint in current_angles:
                    fraction = step / interpolation_steps
                    intermediate_angles[joint] = (
                        current_angles[joint] + angle_differences[joint] * fraction
                    )
            
            limb.set_joint_positions(intermediate_angles)
            rospy.sleep(0.05)
        
        # 最终精确定位
        limb.move_to_joint_positions(
            positions=target_angles,
            timeout=timeout,
            threshold=0.00872665  
        )
        
        rospy.sleep(0.1)
        return True
        
    except Exception as e:
        rospy.logerr(f"平滑运动执行失败: {e}")
        return False


def send_image(path):
    """发送图片到Baxter头部显示屏"""
    img = cv2.imread(path)
    if img is None:
        rospy.logwarn(f"图像未找到: {path}")
        return
    
    img = cv2.resize(img, (1024, 600))
    
    bridge = cv_bridge.CvBridge()
    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    
    rospy.sleep(1.5)


def perform_right_arm_sequence():
    """执行右臂完整运动序列"""
    try:
        right_limb = baxter_interface.Limb('right')
        left_limb = baxter_interface.Limb('left')
        
        rospy.loginfo("开始执行右臂运动序列...")
        
        # 阶段1: 移动到打招呼位置
        rospy.loginfo("阶段1: 移动到打招呼位置")
        move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_ANGLES)
        rospy.sleep(0.5)
        
        # 阶段2: 显示第一张图片
        rospy.loginfo("显示第一张图片")
        send_image("/root/baxter_ws/src/greet_demo/pic/hello.jpg")

        # 阶段3: 执行三次打招呼动作循环
        rospy.loginfo("阶段3: 执行三次打招呼动作循环")
        for i in range(3):
            rospy.loginfo(f"打招呼动作循环 {i+1}/3")
            move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_END_ANGLES)
            rospy.sleep(0.3)
            move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_ANGLES)
            rospy.sleep(0.3)

        # 阶段4: 显示第二张��片
        rospy.loginfo("显示第二张图片")
        send_image("/root/baxter_ws/src/greet_demo/pic/basketball.jpg")
        
        # 阶段5: 模仿拍球动作
        rospy.loginfo("阶段5: 模仿拍球动作")
        move_to_joint_positions_smooth(right_limb, RIGHT_DRIBBLE_PREP_JOINT_ANGLES)
        rospy.sleep(0.25)
        for i in range(4):
            rospy.loginfo(f"拍球动作循环 {i+1}/4")
            move_to_joint_positions_smooth(right_limb, RIGHT_DRIBBLE_DOWN_JOINT_ANGLES)
            rospy.sleep(0.18)
            move_to_joint_positions_smooth(right_limb, RIGHT_DRIBBLE_UP_JOINT_ANGLES)
            rospy.sleep(0.18)
        move_to_joint_positions_smooth(right_limb, RIGHT_DRIBBLE_PREP_JOINT_ANGLES)
        rospy.sleep(0.4)

        # 阶段6: 显示第三张图片
        rospy.loginfo("显示第三张图片")
        send_image("/root/baxter_ws/src/greet_demo/pic/goodbye.jpg")

        # 阶段7: 再次执行三次打招呼动作循环
        rospy.loginfo("阶段7: 再次执行三次打招呼动作循环")
        for i in range(3): 
            move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_END_ANGLES)
            rospy.sleep(0.3)
            move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_ANGLES)
            rospy.sleep(0.3)
        
        # 阶段8: 返回初始位置
        rospy.loginfo("阶段8: 返回初始位置")
        move_to_joint_positions_smooth(right_limb, RIGHT_INITIAL_JOINT_ANGLES)
        move_to_joint_positions_smooth(left_limb, LEFT_INITIAL_JOINT_ANGLES)
        
        rospy.loginfo("右臂运动序列执行完成")
        
    except Exception as e:
        rospy.logerr(f"右臂运动序列执行失败: {e}")


class BaxterDualArmController:
    def __init__(self):
        """初始化Baxter双臂控制系统"""
        rospy.init_node('baxter_dual_arm_controller', anonymous=True)
        
        rospy.sleep(1.0)
        
        # 初始化双臂
        rospy.loginfo("开始初始化双臂...")
        self.left_limb = initialize_left_arm()
        self.right_limb = initialize_right_arm()
        
        rospy.loginfo("Baxter双臂控制系统启动完成")
        
        rospy.on_shutdown(self.cleanup)
    
    def run_right_arm_sequence(self):
        """运行右臂运动序列"""
        rospy.loginfo("开始执行右臂运动流程...")
        perform_right_arm_sequence()
        rospy.loginfo("右臂运动流程执行完成")
    
    def cleanup(self):
        """关闭时停止所有动作"""
        rospy.loginfo("正在关闭Baxter双臂控制系统...")
        try:
            if hasattr(self, 'left_limb'):
                move_to_joint_positions_smooth(self.left_limb, LEFT_INITIAL_JOINT_ANGLES)
            
            if hasattr(self, 'right_limb'):
                move_to_joint_positions_smooth(self.right_limb, RIGHT_INITIAL_JOINT_ANGLES)
                
        except Exception as e:
            rospy.logwarn(f"清理过程中异常: {e}")
        
        rospy.loginfo("系统关闭完成")


if __name__ == "__main__":
    try:
        controller = BaxterDualArmController()
        controller.run_right_arm_sequence()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断异常")
    except Exception as e:
        rospy.logerr(f"系统运行异常: {e}")
    finally:
        rospy.loginfo("程序结束")
```

### 4.3 动作序列流程

```
右臂运动序列流程:

阶段1: 移动到打招呼位置
    └── RIGHT_HELLO_JOINT_ANGLES

阶段2: 显示 hello.jpg

阶段3: 打招呼动作 (3次循环)
    ├──RIGHT_HELLO_JOINT_END_ANGLES
    └──RIGHT_HELLO_JOINT_ANGLES

阶段4: 显示 basketball.jpg

阶段5: 模仿拍球动作 (4次循环)
    ├──RIGHT_DRIBBLE_PREP (准备)
    ├──RIGHT_DRIBBLE_DOWN (下压)
    └──RIGHT_DRIBBLE_UP (上抬)

阶段6: 显示 goodbye.jpg

阶段7: 打招呼动作 (3次循环)

阶段8: 返回初始位置
    └──RIGHT_INITIAL_JOINT_ANGLES
```

### 4.4 平滑移动算法

end.py 采用了分段插值实现平滑运动：

```
当前角度 ──► 线性插值 ──► 30个中间位置 ──► 最终目标位置
          (fraction = step / 30)
```

**特点**：
- 避免关节突变
- 减小机械冲击
- 运动更自然

---

## 5 使用方法

### 5.1 启动 dance.py

```bash
# 启动舞蹈节点
rosrun my_robot_pkg dance.py
```

### 5.2 启动 gripper.py

```bash
# 启动视觉抓取节点
rosrun my_robot_pkg gripper.py

# 发送释放命令
rostopic pub /gripper/release std_msgs/String "data: 'q'" -1
```

### 5.3 启动 end.py

```bash
# 启动双臂动作序列
rosrun my_robot_pkg end.py
```

---

## 6 依赖包

根据 package.xml：

- baxter_core_msgs
- baxter_moveit_config
- controller_manager_msgs
- cv_bridge
- message_generation
- moveit_commander
- roscpp
- rospy
- sensor_msgs
- std_msgs
- std_srvs
- trajectory_msgs
- OpenCV
- NumPy