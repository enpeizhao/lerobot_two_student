import time
import socket
# import pickle
import json  # 使用 JSON 替换 pickle
import logging
import argparse
import numpy as np
np.set_printoptions(precision=6, suppress=True)
# 设置日志输出格式和级别
logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')

class EpisodeAPP:
    def __init__(self, ip='localhost', port=12345):
        """
        初始化客户端，设置服务器的IP地址和端口号。
        """
        self.server_address = (ip, port)

    def send_command(self, command):
        """
        发送命令到服务器并接收返回值。
        
        参数：
            command (dict): 包含动作和参数的命令字典。
        
        返回：
            从服务器接收到的结果（反序列化后的对象），若发生异常则返回 None。
        """
        try:
            # 使用 with 确保 socket 在使用后能正确关闭
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                # 连接到服务器
                client_socket.connect(self.server_address)
                
                # 将命令序列化为字节流（使用 JSON）
                data = json.dumps(command).encode('utf-8')
                data_length = len(data)
                
                # 先发送数据长度（8字节）给服务器，便于服务器知道接收数据的大小
                client_socket.sendall(data_length.to_bytes(8, byteorder='big'))
                # 发送实际的数据
                client_socket.sendall(data)
                
                # 接收服务器返回的数据长度（8字节）
                response_length = client_socket.recv(8)
                if not response_length:
                    return None
                response_length = int.from_bytes(response_length, byteorder='big')
                
                # 根据数据长度接收完整的数据包
                response_data = b''
                while len(response_data) < response_length:
                    packet = client_socket.recv(response_length - len(response_data))
                    if not packet:
                        break
                    response_data += packet
                
                # 将接收到的字节流反序列化为 Python 对象（使用 JSON）
                result = json.loads(response_data.decode('utf-8'))
                return result
        except Exception as e:
            logging.error(f"send_command 出错: {e}")
            return None

    def emergency_stop(self, enable):
        """
        急停或解除急停操作。
        
        参数：
            enable (int): 1 表示急停，0 表示解除急停。
        
        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {'action': 'emergency_stop', 'params': enable}
        return self.send_command(command)

    def angle_mode(self, angles, speed_ratio=1):
        """
        通过角度模式控制电机运动，移动到指定关节角度。
        
        参数：
            angles (list): 各个电机目标角度列表。
            speed_ratio (float): 运动速度比例，默认为 1。
            
        返回：
            服务器返回的预计运动时间（秒）。
        """
        command = {'action': 'angle_mode', 'params': [angles, speed_ratio]}
        return self.send_command(command)
    
    def sync_motor_angles(self):
        """
        同步电机角度
        """
        command = {'action': 'sync_motor_angles', 'params': []}
        return self.send_command(command)

    def move_xyz_rotation(self, position, orientation, rotation_order="zyx", speed_ratio=1):
        """
        通过 XYZ 坐标和欧拉角控制电机运动，采用普通位姿模式。

        参数：
            position (list): 三维空间位置，包含 [x, y, z]。
            orientation (list): 欧拉角，包含 [roll, pitch, yaw]。
            rotation_order (str): 旋转顺序（支持 "zyx" 或 "xyz"，默认 "zyx"）。
            speed_ratio (float): 运动速度比例（0~1之间，默认 1，超过该范围会自动截断）。

        返回值：
            -1 表示 IK 无解；
            >0 表示有解，返回预计运动时间（秒），建议客户端等待该时间后再发送下一条指令。
        """
        params = position + orientation + [rotation_order, speed_ratio]
        print(params)
        command = {'action': 'move_xyz_rotation', 'params': params}
        return self.send_command(command)

    def move_linear_xyz_rotation(self, position, orientation, rotation_order="zyx"):
        """
        采用直线模式，通过 XYZ 坐标和欧拉角控制电机运动。直线模式下无法调整速度，
        运动过程中服务器会先返回运动时间再执行运动操作（可能会阻塞）。

        参数：
            position (list): 三维空间位置，包含 [x, y, z]。
            orientation (list): 欧拉角，包含 [roll, pitch, yaw]。
            rotation_order (str): 旋转顺序（支持 "zyx" 或 "xyz"，默认 "zyx"）。

        返回值：
            -1 表示 IK 求解无解；
            >0 表示有解，返回预计运动时间（秒）。
        """
        params = position + orientation + [rotation_order]
        command = {'action': 'move_linear_xyz_rotation', 'params': params}
        return self.send_command(command)

    def gripper_on(self):
        """
        启动负压吸盘抓取操作。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {'action': 'gripper_on'}
        return self.send_command(command)

    def gripper_off(self):
        """
        负压吸盘释放操作。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {'action': 'gripper_off'}
        return self.send_command(command)

    def servo_gripper(self, angle):
        """
        通过舵机控制夹爪角度。

        参数：
            angle (int): 夹爪角度。

        返回：
            固定返回 1（单位：秒）。
        """
        command = {'action': 'servo_gripper', 'params': angle}
        return self.send_command(command)

    def robodk_simu(self, enable):
        """
        控制 Robodk 模拟开关。

        参数：
            enable (int): 1 表示开启模拟，0 表示关闭模拟。

        返回：
            固定返回 0.05（单位：秒）。
        """
        command = {'action': 'robodk_simu', 'params': enable}
        return self.send_command(command)

    def set_free_mode(self, mode):
        """
        设置电机自由模式。注意：在进入自由模式前需要提醒用户准备好托举。

        参数：
            mode (int): 1 进入自由模式，0 退出自由模式。

        返回：
            固定返回 0.1（单位：秒）。
        """
        command = {'action': 'set_free_mode', 'params': mode}
        return self.send_command(command)

    def get_motor_angles(self):
        """
        获取当前电机的角度信息。

        返回：
            一个长度为 6 的角度列表（单位：度）。由于 CAN 总线阻塞，部分电机角度读取可能失败，此时返回 None。
        """
        command = {'action': 'get_motor_angles'}
        return self.send_command(command)
    
    def dynamic_move(self,goal_pos, current_joint_pulses, motors_max_speed):
        """
        动态移动到目标位置。

        参数：
            goal_pos (list): 目标位置，包含 6 个关节角度。
            current_joint_pulses (list): 当前关节脉冲数，包含 6 个值。
            motors_max_speed (list): 电机最大速度，包含 6 个值。

        返回：
            固定返回 0.05（单位：秒）。
        """
        # goal_pos, current_joint_pulses, motors_max_speed
        command = {'action': 'dynamic_move', 'params': [goal_pos, current_joint_pulses, motors_max_speed]}
        return self.send_command(command)
    
    def get_T(self):
        """
        获取齐次变换矩阵 T。
        """
        command = {'action': 'get_T'}
        result = self.send_command(command)
        if result is None:
            return None
        else:
            # 将列表转换为 numpy 数组并返回
            return np.array(result).reshape(4, 4)
    def get_pose(self,rotation_order="xyz"):
        """
        获取位姿，支持xyz和zyx两种旋转顺序。
        """
        command = {'action': 'get_pose', 'params': rotation_order}
        result = self.send_command(command)
        if result is None:
            return None
        else:
            return np.array(result)
