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
    'right_s0': -0.395,    # 肩部外展
    'right_s1': -0.202,    # 肩部前屈
    'right_e0': 1.831,     # 肘部内旋
    'right_e1': 1.981,     # 肘部伸展
    'right_w0': -1.979,    # 腕部保持中立
    'right_w1': -1.100,    # 腕部伸展
    'right_w2': -0.448     # 腕部旋转初始
}
# 右臂平举上抬位置
RIGHT_RAISE_JOINT_ANGLES = {
    'right_s0': -0.097,    # 肩部外展
    'right_s1': 0.118,    # 肩部前屈
    'right_e0': 0.393,     # 肘部内旋
    'right_e1': -0.692,     # 肘部伸展
    'right_w0': 0.206,    # 腕部保持中立
    'right_w1': 0.055,    # 腕部伸展
    'right_w2': -0.161     # 腕部旋转初始
}

# 右臂模仿拍球动作 - 准备姿态（手在球上方）
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
        
        # 启用机器人（如果未启用）
        if not baxter_interface.RobotEnable().state().enabled:
            baxter_interface.RobotEnable().enable()
            rospy.sleep(1.0)
        
        # 设置关节速度限制
        left_limb.set_joint_position_speed(MOTION_PARAMS['joint_velocity_limit'])
        
        # 平滑移动到自然下垂位置
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
        
        # 设置关节速度限制
        right_limb.set_joint_position_speed(MOTION_PARAMS['joint_velocity_limit'])
        
        # 平滑移动到自然下垂位置
        move_to_joint_positions_smooth(right_limb, RIGHT_INITIAL_JOINT_ANGLES)
        
        rospy.loginfo("右臂初始化完成 - 自然下垂位置")
        return right_limb
        
    except Exception as e:
        rospy.logerr(f"右臂初始化失败: {e}")
        raise

def move_to_joint_positions_smooth(limb, target_angles, timeout=15.0, 
                                   max_step=0.1, interpolation_steps=30):
    """
    平滑移动到目标关节位置，最小化关节角变化
    参数:
        limb: Limb对象
        target_angles: 目标关节角度字典
        timeout: 超时时间（秒）
        max_step: 单步最大角度变化（弧度）
        interpolation_steps: 插值步数
    """
    try:
        # 获取当前关节角度
        current_angles = limb.joint_angles()
        
        # 验证关节名称匹配性
        joint_names = limb.joint_names()
        for joint_name in target_angles.keys():
            if joint_name not in joint_names:
                rospy.logerr(f"关节名称错误: {joint_name} 不属于 {limb.name} 臂")
                rospy.logerr(f"可用关节: {joint_names}")
                return False
        
        # 计算关节角度变化
        angle_differences = {}
        for joint in target_angles:
            if joint in current_angles:
                diff = target_angles[joint] - current_angles[joint]
                angle_differences[joint] = diff
        
        # 采用分段插值实现平滑运动
        for step in range(1, interpolation_steps + 1):
            intermediate_angles = {}
            for joint in target_angles:
                if joint in current_angles:
                    # 线性插值
                    fraction = step / interpolation_steps
                    intermediate_angles[joint] = (
                        current_angles[joint] + angle_differences[joint] * fraction
                    )
            
            # 执行中间位置运动
            limb.set_joint_positions(intermediate_angles)
            rospy.sleep(0.05)  # 短暂延迟确保运动平滑
        
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
    
    # 缩放图片以适应Baxter显示屏(1024x600)
    img = cv2.resize(img, (1024, 600))
    
    # 创建图像消息
    bridge = cv_bridge.CvBridge()
    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    
    # 发布图像
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    
    # 确保图像稳定显示
    rospy.sleep(1.5)

def perform_right_arm_sequence():
    """执行右臂完整运动序列"""
    try:
        # 获取右臂实例
        right_limb = baxter_interface.Limb('right')
        left_limb = baxter_interface.Limb('left')
        
        rospy.loginfo("开始执行右臂运动序列...")
        
        # 阶段1: 从初始位置移动到打招呼位置
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
            
            # 从打招呼位置回到初始位置
            move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_END_ANGLES)
            rospy.sleep(0.3)
            
            # 再次回到打招呼位置
            move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_ANGLES)
            rospy.sleep(0.3)

        # 阶段4: 显示第二张图片
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
            rospy.loginfo(f"第二次打招呼动作循环 {i+1}/3")
            
            # 从打招呼位置回到初始位置
            move_to_joint_positions_smooth(right_limb, RIGHT_HELLO_JOINT_END_ANGLES)
            rospy.sleep(0.3)
            
            # 移动到打招呼位置
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
        
        # 等待ROS核心启动
        rospy.sleep(1.0)
        
        # 初始化双臂
        rospy.loginfo("开始初始化双臂...")
        self.left_limb = initialize_left_arm()
        self.right_limb = initialize_right_arm()
        
        rospy.loginfo("✅ Baxter双臂控制系统启动完成")
        rospy.loginfo("左臂: 自然下垂位置")
        rospy.loginfo("右臂: 自然下垂位置")
        
        # 设置ROS关闭时的清理函数
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
                rospy.loginfo("将左臂移动到安全位置...")
                move_to_joint_positions_smooth(self.left_limb, LEFT_INITIAL_JOINT_ANGLES)
            
            if hasattr(self, 'right_limb'):
                rospy.loginfo("将右臂移动到安全位置...")
                move_to_joint_positions_smooth(self.right_limb, RIGHT_INITIAL_JOINT_ANGLES)
                
        except Exception as e:
            rospy.logwarn(f"清理过程中异常: {e}")
        
        rospy.loginfo("系统关闭完成")

if __name__ == "__main__":
    try:
        # 创建双臂控制系统实例
        controller = BaxterDualArmController()
        
        # 执行右臂运动序列
        controller.run_right_arm_sequence()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断异常")
    except Exception as e:
        rospy.logerr(f"系统运行异常: {e}")
    finally:
        rospy.loginfo("程序结束")
