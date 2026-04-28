# greet_demo 语音交互演示系统技术报告

## 1 项目概述

greet_demo 是基于 ROS (Robot Operating System) 的 Baxter 机器人语音交互演示系统，实现了语音控制机器人动作、语音回复、头部显示屏显示图片等功能。该系统属于 baxter_examples 包的一部分，依赖 pocketsphinx 语音识别模块和 sound_play 语音合成模块。

### 1.1 项目背景

Baxter 是 Rethink Robotics 公司生产的双臂协作机器人，主要用于科研和教育领域。Baxter 机器人拥有两条可独立控制的机械臂，每条手臂有 7 个自由度，末端配有可调节夹爪。同时，Baxter 还配备了头部显示屏，可以显示图像信息。

greet_demo 系统的设计目标是让 Baxter 能够"听懂"人类的语音指令，并做出相应的反应，包括：
- 通过扬声器发出语音回复
- 通过机械臂执行特定的动作
- 通过头部显示屏显示相关图片

这种多模态的交互方式使机器人更加拟人化，更容易与人类进行自然互动。

### 1.2 项目位置
- 路径：`/root/baxter_ws/src/greet_demo/`

### 1.3 核心文件
| 文件 | 功能 | 说明 |
|------|------|------|
| polite.py | 动作回放器 | 读取 .dat 动作文件并控制机械臂 |
| voice_nav.py | 语音导航主程序 | 处理语音命令并协调各种响应 |
| pro/greet.dat | 问候动作数据 | CSV 格式的关节角度数据 |
| pro/shake_hand.dat | 握手动作数据 | CSV 格式的关节角度数据 |
| pic/ | 图片资源目录 | 存储显示用的图片文件 |
| READ_ME | 配置说明 | 原始配置文档 |

---

## 2 系统架构与工作原理

### 2.1 整体架构

greet_demo 系统由多个相互协作的模块组成，每个模块负责特定的功能。系统的设计遵循了 ROS 的分布式架构理念，通过话题（Topics）进行模块间的通信。

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        greet_demo 系统整体架构                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                        用户交互层                                    │  │
│   │  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐        │  │
│   │  │  麦克风  │    │  扬声器  │    │ 显示屏  │    │ 机械臂  │        │  │
│   │  │  输入   │    │  输出   │    │  输出   │    │  执行  │        │  │
│   │  └────┬────┘    └────┬────┘    └────┬────┘    └────┬────┘        │  │
│   └──────┼──────────────┼──────────────┼──────────────┼───────────────┘  │
│          │              │              │              │                    │
│          ▼              ▼              ▼              ▼                    │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                        ROS 通信层                                   │  │
│   │  ┌──────────────────┐  ┌──────────────────┐                     │  │
│   │  │ /recognizer/output │  │    /robot/xdisplay │                     │  │
│   │  │  (std_msgs/String) │  │    (sensor_msgs)   │                     │  │
│   │  └──────────────────┘  └──────────────────┘                     │  │
│   │  ┌──────────────────┐  ┌──────────────────┐                     │  │
│   │  │    /cmd_vel      │  │  /robot/joint_state│                     │  │
│   │  │ (geometry_msgs)  │  │                  │                     │  │
│   │  └──────────────────┘  └──────────────────┘                     │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│          │              │              │              │                         │
│          ▼              ▼              ▼              ▼                    │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                        功能模块层                                    │  │
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │  │
│   │  │ pocketsphinx│  │ sound_play │  │ baxter_     │               │  │
│   │  │ (语音识别)  │  │ (语音合成) │  │ interface   │               │  │
│   │  │             │  │            │  │ (动作控制)  │               │  │
│   │  └──────────────┘  └──────────────┘  └──────────────┘               │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
│          │              │              │                                      │
│          ▼              ▼              ▼                                  │
│   ┌─────────────────────────────────────────────────────────────────────┐  │
│   │                        voice_nav.py (核心控制器)                      │  │
│   │  ┌─────────────────────────────────────────────────────────────┐  │  │
│   │  │  1. 订阅 /recognizer/output 获取语音识别结果                     │  │  │
│   │  │  2. 解析命令关键词                                        │  │  │
│   │  │  3. 协调三种输出响应                                      │  │  │
│   │  └────────────────���────────────────────────────────────────────┘  │  │
│   └─────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 数据流详解

系统的数据流向可以详细描述为以下几个阶段：

#### 2.2.1 语音输入阶段

```
阶段1: 语音采集与识别
─────────────────────────────────────────
用户发出语音命令
        │
        ▼
┌───────────────────┐
│   麦克风设备       │  ◄── 通过 ALSA/PulseAudio 采集
│ (音频输入设备)    │
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐     ┌─────────────────────┐
│ pocketsphinx 节点  │────►│ 语音识别引擎         │
│ (recognizer.py)    │     │ (Pocketsphinx)       │
└─────────┬─────────┘     └─────────────────────┘
          │
          ▼
┌───────────────────┐
│ /recognizer/output │  ◄── ROS 话题发布
│ 话题              │     │ (std_msgs/String)
└───────────────────┘
```

pocketsphinx 是 CMU 开发的开源语音识别系统，专门针对嵌入式和实时应用场景优化。在本系统中，pocketsphinx 节点负责：
- 实时采集麦克风音频
- 使用预训练的语言模型和声学模型进行识别
- 将识别结果以字符串形式发布到 ROS 话题

#### 2.2.2 命令解析阶段

```
阶段2: 命令解析与匹配
─────────────────────────────────────────
/recognizer/output 话题
        │
        ▼
┌───────────────────┐
│ voice_nav.py      │  ◄── 订阅该话题
│ speech_callback() │
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ get_command()     │  ◄── 关键词匹配
│ 方法              │
└─────────┬─────────┘
          │
          ▼
┌───────────────────┐
│ 匹配成功:         │    匹配失败:
│ hello/old/good/   │    返回 None
│ sport             │
└───────────────────┘
```

关键词匹配采用简单的子字符串搜索：

```python
def get_command(self, data):
    text = data.lower()  # 转换为小写
    for (command, keywords) in self.keywords_to_command.items():
        for word in keywords:
            if text.find(word) > -1:  # 检查是否包含关键词
                return command
```

这种方法虽然在精度上不如复杂的自然语言理解，但对于预定义命令集来说简单有效。

#### 2.2.3 响应执行阶段

```
阶段3: 三种响应执行
─────────────────────────────────────────
识别到命令
        │
        ├─────────────────────┬───────��─��───────────┐
        ▼                    ▼                    ▼
   ┌─────────┐          ┌─────────┐          ┌─────────┐
   │ 语音合成│          │ 動作執行│          │ 图像显示│
   │        │          │        │          │        │
   └────┬────┘          └────┬────┘          └────┬────┘
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│ sound_play   │    │ polite.py    │    │ cv2 +       │
│ 库           │    │ 子进程       │    │ xdisplay    │
└───────────────┘    └───────────────┘    └───────────────┘
        │                    │                    │
        ▼                    ▼                    ▼
   ┌───────────┐      ┌───────────┐      ┌───────────┐
   │ 扬声器输出 │      │ 机械臂运动│      │ 头部屏幕 │
   └───────────┘      └───────────┘      └───────────┘
```

**响应类型详细说明**：

1. **语音合成响应**
   - 使用 sound_play 库的 SoundClient
   - 调用 say() 方法将文本转换为语音
   - 通过计算机默认音频设备输出

2. **动作执行响应**
   - 通过 os.system() 启动子进程
   - 子进程中运行 polite.py
   - .dat 文件包含完整的动作序列

3. **图像显示响应**
   - 使用 OpenCV 读取图片
   - 通过 cv_bridge 转换为 ROS 镜像消息
   - 发布到 /robot/xdisplay 话题

---

## 3 核心模块详解

### 3.1 polite.py 动作回放器

#### 3.1.1 功能��述

polite.py 是系统的动作执行模块，负责读取预先录制的动作数据文件，并控制 Baxter 机器人的双臂按照数据精确运动。

**核心功能**：
- 读取 CSV 格式的动作数据文件
- 控制 14 个机械臂关节（每臂 7 个）
- 控制 2 个夹爪的开合
- 支持时间戳控制的动作回放
- 支持回放速度调节

#### 3.1.2 Baxter 机械臂结构

Baxter 机器人每条手臂有 7 个可独立控制的关节，采用反向运动学设计：

**左臂关节**：
| 关节名称 | 部位 | 运动类型 |
|---------|------|---------|
| left_s0 | 肩部旋转 | 回转 (Roll) |
| left_s1 | 肩部俯仰 | 俯仰 (Pitch) |
| left_e0 | 肘部旋转 | 回转 (Roll) |
| left_e1 | 肘部俯仰 | 俯仰 (Pitch) |
| left_w0 | 腕部旋转 | 回转 (Roll) |
| left_w1 | 腕部俯仰 | 俯仰 (Pitch) |
| left_w2 | 腕部扭转 | 扭转 (Twist) |

**右臂关节**：命名方式相同，只是将 left 改为 right。

**夹爪控制**：
- 控制范围：0（完全闭合）~ 100（完全张开）
- 夹爪采用自适应设计，可以抓取不同形状的物体

#### 3.1.3 动作文件格式详解

动作数据存储在 .dat 文件中，采用 CSV 格式，每一行记录一个时间点的所有关节目标位置：

```csv
time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,left_gripper,right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2,right_gripper
0.495,-0.077,-0.999,-1.189,1.938,0.670,1.027,-0.498,100,-0.710,0.320,1.140,1.137,0.389,0.878,-0.898,100
0.496,-0.077,-0.999,-1.189,1.938,0.670,1.027,-0.498,100,-0.710,0.320,1.140,1.137,0.389,0.878,-0.898,100
```

**字段说明**：
- **time**：该动作帧的目标时间戳（秒），从动作开始计时
- **left_s0 ~ left_w2**：左臂 7 个关节的目标角度（弧度）
- **left_gripper**：左夹爪目标位置（0-100）
- **right_s0 ~ right_w2**：右臂 7 个关节的目标角度（弧度）
- **right_gripper**：右夹爪目标位置（0-100）

#### 3.1.4 工作流程详解

polite.py 的执行流程可以细分为以下步骤：

**步骤1：参数解析**

```
命令行输入: python polite.py -f action.dat --speed 1.5

         │
         ▼
┌─────────────────────────┐
│ argparse 解析器        │
│ -f: 动作文件路径     │
│ --speed: 速度倍率    │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 参数验证              │
│ - 检查文件是否存在    │
│ - 检查速度 > 0       │
└─────────────────────────┘
```

**步骤2：ROS 初始化与机器人连接**

```
         │
         ▼
┌─────────────────────────┐
│ rospy.init_node()       │
│ 节点名称: polite_player │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────┐
│ baxter_interface       │
│ - RobotEnable()        │
│ - enable()            │
│ 创建 Limb(left/right)   │
│ 创建 Gripper(left/right)│
└─────────────────────────┘
```

**步骤3：夹爪校准**

```
         │
         ▼
┌─────────────────────────┐
│ left_gripper.calibrate()│
│ right_gripper.        │
│ calibrate()           │
└─────────────────────────┘

注意：首次使用夹爪时必须校准
      否则可能无法正常工作
```

**步骤4：动作回放主循环**

```
┌─────────────────────────┐
│ for row in rows:       │  ◄── 遍历每一行动作数据
└─────────┬─────────────┘
          │
          ▼
    ┌───────────────────┐
    │ 计算时间间隔     │
    │ dt = (now - prev)│
    │     / speed      │
    └───────┬─────────┘
          │
          ▼
    ┌───────────────────┐
    │ rospy.sleep(dt)   │  ◄── 等待到指定时间
    └───────┬─────────┘
          │
          ▼
    ┌───────────────────┐
    │ 构建关节命令字典   │
    │ left_cmd = {...}  │
    │ right_cmd = {...}│
    └───────┬─────────┘
          │
          ▼
    ┌───────────────────┐
    │ set_joint_       │
    │ positions()      │  ◄── 发送命令到机器人
    └───────┬─────────┘
          │
          ▼
    ┌───────────────────┐
    │ set_gripper()    │  ◄── 控制夹爪
    └───────┬─────────┘
          │
          └──► 循环结束 ◄────
```

#### 3.1.5 关键代码解析

```python
# 关节名称常量定义
LEFT_JOINTS = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
RIGHT_JOINTS = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]

# 动作回放核心循环
for row in rows:
    # 1. 时间控制：根据时间戳计算等待时间
    now_t = safe_float(row.get("time", 0.0), 0.0)
    if prev_t is not None:
        dt = max(0.0, (now_t - prev_t) / args.speed)
        if dt > 0:
            rospy.sleep(dt)
    prev_t = now_t
    
    # 2. 构建左臂命令字典：从CSV行提取7个关节角度
    left_cmd = {j: safe_float(row.get(j, 0.0), 0.0) for j in LEFT_JOINTS}
    
    # 3. 构建右臂命令字典
    right_cmd = {j: safe_float(row.get(j, 0.0), 0.0) for j in RIGHT_JOINTS}
    
    # 4. 发送关节位置命令到机器人
    left_limb.set_joint_positions(left_cmd)
    right_limb.set_joint_positions(right_cmd)
    
    # 5. 夹爪控制（可选）
    maybe_set_gripper(left_gripper, row.get("left_gripper", "100"))
    maybe_set_gripper(right_gripper, row.get("right_gripper", "100"))
```

**代码解析**：
- set_joint_positions() 是非阻塞调用，立即返回
- 机器人控制器会自动规划轨迹
- 实际运动时间可能略长于时间戳间隔
- 下一帧命令会覆盖当前命令

---

### 3.2 voice_nav.py 语音导航主程序

#### 3.2.1 功能描述

voice_nav.py 是整个系统的核心控制器，负责接收语音命令并协调各种响应。

**主要功能**：
- 订阅语音识别话题获取命令
- 关键词匹配识别用户意图
- 协调语音合成、动作执行、图像显示三种输出

#### 3.2.2 命令映射表详解

系统预设了以下命令映射，每条命令可以触发多种关键词：

| 命令 | 触发关键词 | 语音回复 | 动作 | 显示图片 |
|------|-----------|---------|------|---------|
| hello | hello, hello baxter, hi | "hello" | 握手动作 | 1.png |
| old | old, how old, how | "i am one year old" | 无 | old.jpg |
| good | good, good bye, bye | "bye bye" | 挥手动作 | goodbye.jpg |
| sport | what is, sport | "my favorite sport is basketball" | 无 | basketball.jpg |

**关键词匹配说明**：
- 关键词按优先级顺序匹配
- 第一个匹配的关键词决定响应类型
- 不区分大小写
- 支持模糊匹配（包含即可）

#### 3.2.3 三种输出响应详解

**（1）语音合成响应**

使用 sound_play 库进行语音合成：

```python
# 初始化
self.voice = rospy.get_param("~voice", "voice_kal_diphone")
self.soundhandle = SoundClient()

# 调用
def talkback(self, word):
    rospy.loginfo(word)  # 打印到日志
    self.soundhandle.say(word, self.voice)  # 语音合成
```

- voice_kal_diphone 是默认音色
- 支持多种可选音色（通过参数配置）
- 音频输出到计算机默认音频设备

**（2）动作执行响应**

通过 os.system() 启动子进程：

```python
# 构建命令字符串
pypath = "python " + sys.path[0] + '/polite.py' + " -f " + sys.path[0] + "/pro/shake_hand.dat"
os.system(pypath)  # 执行
```

- 启动新的 Python 进程
- 独立于主进程运行
- 可以并行处理多个动作请求
- 注意：子进程会阻塞直到动作完成

**（3）图像显示响应**

将图片发送到 Baxter 头部显示屏：

```python
def send_image(path):
    img = cv2.imread(path)  # 读取图片
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")  # 转换
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)  # 发布
    rospy.sleep(1)  # 等待发布完成
```

- 支持 JPEG/PNG 等常见格式
- 通过 /robot/xdisplay 话题发布
- latch=True 确保消息被保留
- Baxter 显示屏分辨率：1024 x 600

#### 3.2.4 初始化流程

voice_nav.py 启动时的初始化流程：

```
┌──────────────────────────────────────────┐
│ __init__() 初始化                         │
├──────────────────────────────────────────┤
│                                          │
│ 1. rospy.init_node('voice_nav')           │
│    - 注册节点到 ROS master                 │
│                                          │
│ 2. rospy.on_shutdown(cleanup)            │
│    - 设置关闭时的清理函数                  │
│                                          │
│ 3. 创建发布者                             │
│    - cmd_vel (geometry_msgs.Twist)        │
│    - 用于控制移动基座（本文中未使用）     │
│                                          │
│ 4. 创建订阅者                             │
│    - /recognizer/output (std_msgs/String) │
│    - 回调函数: speech_callback            │
│                                          │
│ 5. 创建语音客户端                         │
│    - SoundClient()                       │
│    - voice_kal_diphone                    │
│                                          │
│ 6. 设置频率                               │
│    - self.rate = 5 Hz                    │
│    - 持续发布 cmd_vel                    │
│                                          │
└──────────────────────────────────────────┘
          │
          ▼
┌──────────────────────────────────────────┐
│ while not rospy.is_shutdown():            │
│                                          │
│ 保持发布 cmd_vel (空消息)                 │
│ 这是一个占位循环，防止节点退出           │
│ 实际工作由回调函数完成                   │
└──────────────────────────────────────────┘
```

#### 3.2.5 命��处理详细流程

```
speech_callback(msg) 处理流程
─────────────────────────────────────────
收到 /recognizer/output 消息
        │
        ▼
┌─────────────────┐
│ msg.data       │  ◄── 识别文本
│ "hello baxter" │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ get_command()   │
│ 匹配关键词     │
└────────┬────────┘
         │
    ┌────┴────┐
    ▼         ▼
 ┌────┐   返回 None
 │hello│
 └──┬─┘
    │
    ▼
┌─────────────────────────┐
│ 命令: hello              │
├─────────────────────────┤
│                         │
│ 1. talkback("hello")    │
│    └─► 语音合成输出     │
│                         │
│ 2. send_image("1.png") │
│    └─► 头部显示屏       │
│                         │
│ 3. os.system(polite)    │
│    └─► 执行握手动作     │
│                         │
└─────────────────────────┘
```

---

## 4 ROS 话题与服务

### 4.1 系统使用的 ROS 话题

| 话题名称 | 类型 | 方向 | 说明 |
|---------|------|------|------|
| /recognizer/output | std_msgs/String | 订阅 | 语音识别结果 |
| /robot/xdisplay | sensor_msgs/Image | 发布 | 头部显示屏图像 |
| /cmd_vel | geometry_msgs/Twist | 发布 | 移动基座速度（本系统未使用） |

### 4.2 关键 ROS 参数

| 参数名称 | 默认值 | 说明 |
|---------|-------|------|
| ~rate | 5 | 主循环频率 |
| ~voice | voice_kal_diphone | 语音合成音色 |
| ~source | alsasrc/pulsesrc | 音频输入源 |

---

## 5 使用方法

### 5.1 启动步骤

**步骤1：启动语音识别**

```bash
roslaunch pocketsphinx demo/voice_cmd.launch
```

这个命令会启动 pocketsphinx 语音识别节点，包括：
- ALSA 音频源配置
- 语言模型加载
- 词典文件加载
- 识别引擎初始化

**步骤2：启动语音合成**

```bash
rosrun sound_play soundplay_node.py
```

sound_play 节点负责：
- 接收语音合成请求
- 使用 GStreamer 合成音频
- 输出到扬声器

**步骤3：启动关节轨迹服务器**

```bash
rosrun baxter_interface joint_trajectory_action_server.py
```

Baxter 需要轨迹服务器来处理：
- 关节轨迹规划
- 轨迹执行
- 碰撞检测

**步骤4：启动语音导航**

```bash
rosrun greet_demo voice_nav.py
```

这是主控制器，启动后：
- 订阅语音识别结果
- 等待命令输入
- 协调各种响应

### 5.2 测试方法

1. 确认所有节点已启动（4个终端）
2. 对着麦克风说出命令词
3. 观察机器人响应：
   - 是否有语音回复
   - 是否显示图片
   - 是否执行动作

---

## 6 常见问题与解决方案

### 6.1 gconfaudiosrc 错误

**问题描述**：

运行时出现错误：
```
gi.repository.GLib.Error: gst_parse_error: no element "gconfaudiosrc"
(gst_parse_error): no element "gconfaudiosrc" (1)
```

**原因分析**：

gconfaudiosrc 是旧版 GStreamer 0.10 的音频源插件，用于从 GConf 配置中读取音频设备。在 2009 年之后，GStreamer 1.0 版本中已经移除了这个插件。

**pocketsphinx 默认配置（在 recognizer.py 中）**：

```python
# 默认使用 gconfaudiosrc
self.launch_config = 'gconfaudiosrc'
self.launch_config += " ! audioconvert ! audioresample ! pocketsphinx name=asr ! fakesink"
```

**解决方案**：

1. **方案一：安装缺失插件**
   ```bash
   apt-get update
   apt-get install -y python3-gst-1.0 gstreamer1.0-plugins-good gstreamer1.0-plugins-base
   ```

2. **方案二：使用 alsasrc 替代（推荐）**
   
   修改 launch 文件：
   ```xml
   <launch>
     <node name="recognizer" pkg="pocketsphinx" type="recognizer.py" output="screen">
       <param name="lm" value="$(find pocketsphinx)/demo/voice_cmd.lm"/>
       <param name="dict" value="$(find pocketsphinx)/demo/voice_cmd.dic"/>
       <param name="source" value="alsasrc"/>  <!-- 修改这里 -->
     </node>
   </launch>
   ```

   推荐直接使用已有的 voice_cmd.launch 文件，它已经配置了 alsasrc。

### 6.2 麦克风无法识别

**可能原因**：
- 麦克风设备未正确设置
- 权限问题
- 采样率不匹配

**排查步骤**：

1. 检查音频设备：
   ```bash
   pacmd list-sources  # 查看可用输入设备
   arecord -l        # 列出录音设备
   ```

2. 检查权限：
   ```bash
   sudo usermod -a -G audio $USER
   # 重新登录生效
   ```

3. 测试麦克风：
   ```bash
   arecord -f cd -d 10 test.wav
   aplay test.wav
   ```

### 6.3 语音识别率低

**可能原因**：
- 语言模型不匹配
- 环境噪声
- 麦克风质量

**优化方法**：

1. 使用专用语言模型（针对特定词汇训练）
2. 增加关键词列表中的词汇
3. 调整麦克风位置和增益

### 6.4 动作执行失败

**可能原因**：
- 机器人未连接
- 夹爪未校准
- 动作文件格式错误

**排查步骤**：

```bash
# 检查机器人连接
rosrun baxter_tools enable_robot.py -a

# 手动测试夹爪
rosrun baxter_interface gripper_action.py left calibrate
rosrun baxter_interface gripper_action.py right calibrate

# 检查动作文件
head -n 5 pro/greet.dat
```

---

## 7 建议的 launch 文件

为简化启动流程，建议创建一个统一的 launch 文件：

```xml
<launch>
  <!-- 描述: greet_demo 启动文件 -->
  
  <!-- 语音识别节点 -->
  <node name="recognizer" 
        pkg="pocketsphinx" 
        type="recognizer.py" 
        output="screen">
    <param name="lm" value="$(find pocketsphinx)/demo/voice_cmd.lm"/>
    <param name="dict" value="$(find pocketsphinx)/demo/voice_cmd.dic"/>
    <!-- 使用 alsasrc 替代已废弃的 gconfaudiosrc -->
    <param name="source" value="alsasrc"/>
  </node>
  
  <!-- 语音合成节点 -->
  <node name="soundplay" 
        pkg="sound_play" 
        type="soundplay_node.py" 
        output="screen"/>
  
  <!-- 关节轨迹服务器 -->
  <node name="joint_trajectory_action_server" 
        pkg="baxter_interface" 
        type="joint_trajectory_action_server.py" 
        output="screen"/>
  
  <!-- 语音导航主程序 -->
  <node name="voice_nav" 
        pkg="greet_demo" 
        type="voice_nav.py" 
        output="screen">
    <param name="voice" value="voice_kal_diphone"/>
  </node>
  
</launch>
```

保存为 greet_demo.launch 后使用：
```bash
roslaunch greet_demo greet_demo.launch
```

---

## 8 技术要点总结

### 8.1 ROS 通信机制

- **话题通信**：使用 std_msgs/String 传输识别文本，使用 sensor_msgs/Image 传输图像
- **参数服务器**：存储配置参数，例如音色、音频源等
- **节点管理**：支持节点的启动、停止和监控

### 8.2 多模态交互设计

系统整合了三种输出模态：
- **听觉**：语音合成（sound_play）
- **视觉**：头部显示屏（xdisplay）
- **触觉**：机械臂运动（polite.py）

这种设计使机器��交��更加自然和拟人化。

### 8.3 子进程解耦

使用 os.system() 调用外部脚本实现模块解耦：
- 主进程不阻塞
- 可以并行处理多个请求
- 易于扩展新功能

### 8.4 时间控制精确性

通过时间戳差值计算等待时间：
- 支持任意时间精度的动作
- 可调节回放速度
- 适合复现复杂动作序列

### 8.5 夹爪控制

0-100 范围的控制：
- 简单直观
- 自适应抓取
- 支持不同物体

---

## 9 扩展建议

### 9.1 增加更多命令

可以在 voice_nav.py 中扩展 keywords_to_command 字典：

```python
self.keywords_to_command = {
    'hello': ['hello', 'hello baxter', 'hi'],
    'old': ['old', 'how old', 'how'],
    'good': ['good', 'good bye', 'bye'],
    'sport': ['what is', 'sport'],
    # 添加新命令
    'name': ['what is your name', 'name'],
    'color': ['what color', 'color'],
    'dance': ['dance', 'do a dance'],
}
```

### 9.2 添加新动作

1. 录制新动作：
   - 手动控制机器人执行动作
   - 使用 rosrecord 记录关节状态
   - 导出为 .dat 文件

2. 添加响应：
   ```python
   elif command == 'dance':
       send_image('dance.jpg')
       pypath = "python " + sys.path[0] + '/polite.py' + " -f " + sys.path[0] + "/pro/dance.dat"
       os.system(pypath)
   ```

### 9.3 改善语音识别

1. 使用自定义语言模型
2. 添加静音检测
3. 调整识别阈值

---

## 10 参考资源

- ROS Wiki - pocketsphinx: http://www.ros.org/wiki/pocketsphinx
- Baxter Interface 文档
- ROS by Example Indigo Volume 1
- GStreamer 文档: https://gstreamer.freedesktop.org/documentation/

---

## 11 附录

### 附录A：.dat 文件字段索引

| 列号 | 字段名 | 数据类型 | 范围 |
|-----|--------|---------|------|
| 0 | time | float | >= 0 |
| 1-7 | left_*_joint | float | rad |
| 8 | left_gripper | float | 0-100 |
| 9-15 | right_*_joint | float | rad |
| 16 | right_gripper | float | 0-100 |

### 附录B：常用 ROS 命令

```bash
# 查看话题列表
rostopic list

# 查看话题内容
rostopic echo /recognizer/output

# 查看节点列表
rosnode list

# 查看节点信息
rosnode info /voice_nav

# 查看参数列表
rosparam list
```