#!/usr/bin/env python

"""
让episode1运动到默认位置，方便安装相机、夹爪

使用方法：
python -m lerobot.episode_default_position --ip_left=192.168.1.100 --port_left=8000 --ip_right=192.168.1.101 --port_right=8000
"""

import argparse
import logging
import time

from lerobot.robots.enpei_follower.episode_server import EpisodeAPP
from lerobot.utils.utils import init_logging


def move_to_default_position(ip_left="192.168.1.100", port_left=8000, 
                            ip_right="192.168.1.101", port_right=8000):
    """
    移动机械臂到默认位置
    
    参数:
        ip_left (str): 左侧机械臂的IP地址
        port_left (int): 左侧机械臂的端口
        ip_right (str): 右侧机械臂的IP地址
        port_right (int): 右侧机械臂的端口
    """
    try:
        # 初始化控制器
        controller_left = EpisodeAPP(ip=ip_left, port=port_left)
        controller_right = EpisodeAPP(ip=ip_right, port=port_right)
        logging.info(f"Connected to EnpeiRobot controllers: Left({ip_left}:{port_left}), Right({ip_right}:{port_right})")

        # 移到默认位置
        alpha = 60
        # 第二组位置
        fixed_degrees = [180+alpha, 90, 83, 210, 110-90, 210, 90]
        # 左侧
        t1 = controller_left.angle_mode(fixed_degrees)
        fixed_degrees = [180-alpha, 90, 83, 210, 110-90, 210, 90]
        # 右侧
        t2 = controller_right.angle_mode(fixed_degrees)
        # 休息最大时间
        time.sleep(max(t1,t2))
        logging.info("已经运动到初始位置")

        # 夹爪运动到10度
        controller_left.servo_gripper(40)
        controller_right.servo_gripper(40)
      
        logging.info("Successfully moved to default position")
        
    except Exception as e:
        logging.error(f"Failed to move to default position: {e}")
        raise

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="移动机械臂到默认位置")
    parser.add_argument("--ip_left", type=str, default="192.168.1.100", help="左侧机械臂的IP地址")
    parser.add_argument("--port_left", type=int, default=8000, help="左侧机械臂的端口")
    parser.add_argument("--ip_right", type=str, default="192.168.1.101", help="右侧机械臂的IP地址")
    parser.add_argument("--port_right", type=int, default=8000, help="右侧机械臂的端口")
    args = parser.parse_args()
    
    # 初始化日志
    init_logging()
    
    # 移动到默认位置
    move_to_default_position(
        ip_left=args.ip_left,
        port_left=args.port_left,
        ip_right=args.ip_right,
        port_right=args.port_right
    )

if __name__ == "__main__":
    main()