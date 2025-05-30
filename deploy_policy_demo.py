#coding=utf-8
import os
import numpy as np
import cv2
import h5py
import socket
import argparse
import rospy

from cv_bridge import CvBridge
import pickle
from std_msgs.msg import Header
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
import pyrealsense2 as rs
import numpy as np
import cv2
import queue
import time
import sys
from utils.pkl2hdf5 import process_folder_to_hdf5_video
import threading
from collections import deque

class RealSenseCam:
    def __init__(self, serial_number, name):
        self.serial_number = serial_number
        self.name = name
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(serial_number)
        # 只启用彩色图像流
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 使用双端队列替换队列，简化帧管理
        self.frame_buffer = deque(maxlen=1)  # 仅保留最新一帧
        self.keep_running = False
        self.thread = None
        self.exit_event = threading.Event()

    def start(self):
        self.keep_running = True
        self.exit_event.clear()
        self.pipeline.start(self.config)
        self.thread = threading.Thread(target=self._update_frames)
        self.thread.daemon = True  # 设置为守护线程
        self.thread.start()

    def _update_frames(self):
        try:
            while not self.exit_event.is_set():
                # 等待彩色帧数据（超时5秒）
                frames = self.pipeline.wait_for_frames(5000)
                color_frame = frames.get_color_frame()
                
                if color_frame:
                    # 转换为NumPy数组并存储
                    color_image = np.asanyarray(color_frame.get_data())[:, :, ::-1]
                    self.frame_buffer.append(color_image)  # 保留最新帧
        except Exception as e:
            print(f"Error from {self.name} camera: {e}")
        finally:
            self.pipeline.stop()

    def get_latest_image(self):
        if self.frame_buffer:
            return self.frame_buffer[-1]  # 返回最新一帧
        return None

    def stop(self):
        self.exit_event.set()
        self.keep_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)  # 等待线程安全退出
        self.pipeline.stop()

def main(args):
    rospy.init_node("replay_node")
    
    puppet_arm_left_publisher = rospy.Publisher(args.puppet_arm_left_topic, JointState, queue_size=10)
    puppet_arm_right_publisher = rospy.Publisher(args.puppet_arm_right_topic, JointState, queue_size=10)
    
    master_arm_left_publisher = rospy.Publisher(args.master_arm_left_topic, JointState, queue_size=10)
    master_arm_right_publisher = rospy.Publisher(args.master_arm_right_topic, JointState, queue_size=10)
    
    joint_state_msg = JointState()
    joint_state_msg.header =  Header()
    joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称

    rate = rospy.Rate(args.frame_rate)
    # print(actions)

    cameras = [
        RealSenseCam('337322073280', 'right_camera'),
        RealSenseCam('337322074191', 'head_camera'),
        RealSenseCam('337122072617', 'left_camera')
    ]

    # 启动所有相机
    for cam in cameras:
        cam.start()
    
    for i in range(20): # 热一下相机
        print(f'warm up: {i}', end='\r')
        for cam in cameras:
            color_image = cam.get_latest_image()

    limit = 800
    last_action = np.array([0, 0, 0, 0, 0, 0, 0.068, 0, 0, 0, 0, 0, 0, 0.068], dtype=np.float32)

    print('reset')
    for j in range(200):
        actions = last_action.copy()

        for i in range(actions.shape[0]):
            joint_state_msg.position = actions[:7]
            master_arm_left_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[7:]
            master_arm_right_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[:7]
            puppet_arm_left_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[7:]
            puppet_arm_right_publisher.publish(joint_state_msg)

    last_obs = None

    model = make_policy() # TODO

    while limit > 0:
        print("left limit: ", limit)
        try:
            cur_timestamp = rospy.Time.now()  # 设置时间戳
                    
            joint_state_msg.header.stamp = cur_timestamp 

            data = {"observation": {}, "joint_action": None}
            flag = True
            for cam in cameras:
                color_image = cam.get_latest_image()
                if color_image is not None:
                    data['observation'][cam.name] = {'rgb': color_image}
                else:
                    flag = False
            if not flag:    
                print('missing camera')
                continue

            data['joint_action'] = last_action
            obs = {}

            first_flag = False
            if last_obs is None:
                last_obs = data
                first_flag = True
            
            obs = {"last": last_obs, "current": data, "flag": first_flag}

            result = model.get_action(obs) # TODO

            print("Received result:", result)

        except Exception as e:
            print("Error:", e)
            continue


        actions = result.copy()

        step_num = 60
        for i in range(1, min(actions.shape[0], step_num)):

            if i == min(actions.shape[0], step_num) - 1:
                try:
                    time.sleep(0.2)
                    cur_timestamp = rospy.Time.now()  # 设置时间戳
                            
                    joint_state_msg.header.stamp = cur_timestamp 

                    data = {"observation": {}, "joint_action": None}
                    for cam in cameras:
                        color_image = cam.get_latest_image()
                        if color_image is not None:
                            data['observation'][cam.name] = {'rgb': color_image}
                            
                    data['joint_action'] = last_action
                    last_obs = data
                    
                except Exception as e:
                    print("Error:", e)
            
            # 部署关节动作
            joint_state_msg.position = actions[i][:7]
            master_arm_left_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[i][7:]
            master_arm_right_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[i][:7]
            puppet_arm_left_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[i][7:]
            puppet_arm_right_publisher.publish(joint_state_msg)

            last_action = actions[i]
            time.sleep(0.15)
            rate.sleep() 

            limit -= 1

    print("\033[92m!!!! Finish\033[0m")
        
if __name__ == '__main__':
    os.system("bash scripts/reset_arm.sh")
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', action='store', type=str, help='Task name.',
                        default="aloha_mobile_dummy", required=False)
    parser.add_argument('--setting', action='store', type=str, help='Task name.',
                        default="aloha_mobile_dummy", required=False)
    parser.add_argument('--idx', action='store', type=str, help='Task name.',
                        default="aloha_mobile_dummy", required=False)
    
    parser.add_argument('--camera_names', action='store', type=str, help='camera_names',
                        default=['cam_high', 'cam_left_wrist', 'cam_right_wrist'], required=False)
    
    parser.add_argument('--img_front_topic', action='store', type=str, help='img_front_topic',
                        default='/camera_f/color/image_raw', required=False)
    parser.add_argument('--img_left_topic', action='store', type=str, help='img_left_topic',
                        default='/camera_l/color/image_raw', required=False)
    parser.add_argument('--img_right_topic', action='store', type=str, help='img_right_topic',
                        default='/camera_r/color/image_raw', required=False)
    
    parser.add_argument('--master_arm_left_topic', action='store', type=str, help='master_arm_left_topic',
                        default='/master/joint_left', required=False)
    parser.add_argument('--master_arm_right_topic', action='store', type=str, help='master_arm_right_topic',
                        default='/master/joint_right', required=False)
    
    parser.add_argument('--puppet_arm_left_topic', action='store', type=str, help='puppet_arm_left_topic',
                        default='/puppet/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_topic', action='store', type=str, help='puppet_arm_right_topic',
                        default='/puppet/joint_right', required=False)
    
    parser.add_argument('--robot_base_topic', action='store', type=str, help='robot_base_topic',
                        default='/cmd_vel', required=False)
    parser.add_argument('--use_robot_base', action='store', type=bool, help='use_robot_base',
                        default=False, required=False)
    
    parser.add_argument('--frame_rate', action='store', type=int, help='frame_rate',
                        default=30, required=False)
    
    parser.add_argument('--only_pub_master', action='store_true', help='only_pub_master',required=False)

    args = parser.parse_args()
    main(args)