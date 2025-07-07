#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

# 1. 导入关键模块
#    - Server: 这是启动动态调参服务的核心类
#    - rqt_tutorialConfig: 这是catkin_make根据你的.cfg文件自动生成的Python类！它的名字是根据 gen.generate(..., "rqt_tutorial") 来的。
from dynamic_reconfigure.server import Server
from rqt_tutorial.cfg import rqt_tutorialConfig

# 定义动态调参器的类
class DynamicParamNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('rqt_tutorial_node', anonymous=False)
        
        # 初始化将要被动态调整的参数变量
        # 给它们一个初始值，但这些值很快会被.cfg文件中的默认值覆盖（只是为了提高代码可读性，在 __init__ 中就能清楚地看到这个类有哪些重要的成员变量，以及他们的初始值）
        self.speed_a = 0.0
        self.speed_b = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0
        
        rospy.loginfo("rqt_tutorial节点已启动，等待参数更新...")

        # 2. 创建动态调参服务实例 (关键一步!)
        #    - 第一个参数是自动生成的Config类
        #    - 第二个参数是回调函数，当参数被修改时，会自动调用 self.reconfigure_callback
        #    当这行代码执行时，回调函数会立刻被调用一次，用.cfg中的默认值来初始化所有参数
        self.srv = Server(rqt_tutorialConfig, self.reconfigure_callback)

        # 启动主循环，用于持续打印参数值
        # 创建类的实例时，会自动执行__init__方法中的代码，如果想在执行完__init__后继续执行其他逻辑，可以在这里调用一个run方法（代码的规范即是在实例的最后定义run函数）
        self.run()

    # 3. 编写回调函数 (参数更新时的响应函数)
    def reconfigure_callback(self, config, level):
        """当rqt_reconfigure中的参数被改变时，此函数被自动调用"""
        
        # level参数是一个位掩码，用于指示哪个参数组被改变了，我们暂时用不到它（我们的逻辑是参数一变就全同步，而并非“如果A组参数变化才同步，其他组变化不同步”这种按组做不同处理的逻辑）
        
        # 将接收到的新参数值(config对象)赋给类的成员变量
        # config对象的成员变量名，就是你在.cfg文件中定义的参数名！
        self.speed_a = config.speed_A
        self.speed_b = config.speed_B
        self.x_pos = config.X_position
        self.y_pos = config.Y_position

        # 打印一条日志，让我们知道回调函数被触发了，并显示更新后的值
        rospy.loginfo("""参数已更新:
        Speed A: {speed_A}
        Speed B: {speed_B}
        Position X: {X_position}
        Position Y: {Y_position}
        """.format(**config))
        
        # 注意：回调函数必须返回config对象，这是API的要求
        return config

    def run(self):
        """
        节点的主循环，只是为了打印参数，让我们看到效果
        run函数是在__init__函数最后调用的，这样可以确保在节点启动后初始化完成以后进入run函数
        """
        rate = rospy.Rate(1) # 1 Hz (每秒打印一次)
        while not rospy.is_shutdown():
            rospy.loginfo("--- 当前参数值 ---")
            rospy.loginfo(f"速度A: {self.speed_a:.2f}, 速度B: {self.speed_b:.2f}")
            rospy.loginfo(f"坐标X: {self.x_pos:.2f}, 坐标Y: {self.y_pos:.2f}")
            rospy.loginfo("---------------------")
            rate.sleep()

if __name__ == "__main__":
    try:
        DynamicParamNode()
    except rospy.ROSInterruptException:
        pass