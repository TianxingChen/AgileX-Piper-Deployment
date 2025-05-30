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

import queue
import time
import sys
sys.path.append('./')
from utils.pkl2hdf5 import process_folder_to_hdf5_video
import cv2

from utils.real_sense_cam import RealSenseCam
from utils.ros_operator import RosOperator

def load_tran_data(file_path):
    with open(file_path, 'rb') as f:
        traj_data = pickle.load(f)
    return traj_data

def load_robotwin_hdf5_joint(dataset_path):
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        vector = root['/joint_action/vector'][()]
    return vector

def save_to_pkl(obj, filename):
    with open(filename, 'wb') as f:
        pickle.dump(obj, f)
    # print(f"对象已保存到 {filename}")

def send_object_to_server(obj, host='localhost', port=10948):
    # 序列化对象
    data = pickle.dumps(obj)
    
    # 创建socket连接
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        print(f"Connected to server {host}:{port}")
        
        # 发送数据
        s.sendall(data)
        print("Object sent to server")
        
        # 关闭写入端，让服务器知道数据已发送完
        s.shutdown(socket.SHUT_WR)
        
        # 接收结果
        result_data = b''
        while True:
            chunk = s.recv(4096)
            if not chunk:  # 接收到空数据表示结束
                break
            result_data += chunk
            print(f"Received chunk of size: {len(chunk)}")
        
        # 反序列化结果
        try:
            result = pickle.loads(result_data)
            if isinstance(result, dict) and 'error' in result:
                raise Exception(f"Server error: {result['error']}")
            return result
        except Exception as e:
            raise Exception(f"Error receiving result: {e}")

def main(args):
    ros_operator = RosOperator(args)
    # rospy.init_node("replay_node")
    
    # puppet_arm_left_publisher = rospy.Publisher(args.puppet_arm_left_topic, JointState, queue_size=10)
    # puppet_arm_right_publisher = rospy.Publisher(args.puppet_arm_right_topic, JointState, queue_size=10)
    
    master_arm_left_publisher = rospy.Publisher(args.master_arm_left_topic, JointState, queue_size=10)
    master_arm_right_publisher = rospy.Publisher(args.master_arm_right_topic, JointState, queue_size=10)
    
    joint_state_msg = JointState()
    joint_state_msg.header =  Header()
    joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称

    rate = rospy.Rate(args.frame_rate)

    # cameras = [
    #     RealSenseCam('337322073280', 'right_camera'),
    #     RealSenseCam('337322074191', 'head_camera'),
    #     RealSenseCam('337122072617', 'left_camera')
    # ]

    # 启动所有相机
    # for cam in cameras:
    #     cam.start()
    
    # for i in range(20):
    #     print(f'warm up: {i}', end='\r')
    #     for cam in cameras:
    #         color_image = cam.get_latest_image()
    #     time.sleep(0.15)

    step_limit = 800
    last_action = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1], dtype=np.float32)

    for j in range(200):
        actions = last_action.copy()
        actions[6] = last_action[6] * 0.88 - 0.22
        actions[13] = last_action[13] * 0.88 - 0.22

        for i in range(actions.shape[0]):
            joint_state_msg.position = actions[:7]
            master_arm_left_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[7:]
            master_arm_right_publisher.publish(joint_state_msg)

            # joint_state_msg.position = actions[:7]
            # puppet_arm_left_publisher.publish(joint_state_msg)

            # joint_state_msg.position = actions[7:]
            # puppet_arm_right_publisher.publish(joint_state_msg)
    print('Reset')

    last_obs = None
    while step_limit > 0:
        print("left step_limit: ", step_limit)
        try:
            cur_timestamp = rospy.Time.now()  # 设置时间戳
                    
            joint_state_msg.header.stamp = cur_timestamp 

            data = {"observation": {}, "joint_action": None}
            

            # Real Sense
            # for cam in cameras:
            #     color_image = cam.get_latest_image()
            #     if color_image is not None:
            #         data['observation'][cam.name] = {'rgb': color_image}
            #     else:
            #         flag = False
            # if not flag:    
            #     print('missing camera')
            #     continue

            # Real Sense
            # flag = False
            # while not flag:
            #     cnt = 0
            #     for cam in cameras:
            #         color_image = cam.get_latest_image()
            #         if color_image is not None:
            #             cnt += 1
            #             data['observation'][cam.name] = {'rgb': color_image}
            #             # if cam.name == 'head_camera':
            #             #     cv2.imshow(cam.name, color_image)
            #             #     key = cv2.waitKey(1)
            #     if cnt == 3:
            #         flag = True

            img_front, img_left, img_right, img_front_depth, img_left_depth, \
            img_right_depth, puppet_arm_left, puppet_arm_right, robot_base = ros_operator.save_get_frame()
            data['observation'] = {'head_camera': {'rgb': img_front}, 'left_camera': {'rgb': img_left}, 'right_camera': {'rgb': img_right}}

            data['joint_action'] = last_action
            message = {}

            first_flag = False
            if last_obs is None:
                last_obs = data
                first_flag = True
            
            message= {"last": last_obs, "current": data, "flag": first_flag}

            result = send_object_to_server(message)

        except Exception as e:
            print("Error:", e)
            continue


        actions = result.copy()

        actions[:, 6] = actions[:, 6] * 0.88 - 0.22
        actions[:, 13] = actions[:, 13] * 0.88 - 0.22

        step_num = 30
        for i in range(1, min(actions.shape[0], step_num)):

            if i == min(actions.shape[0], step_num) - 1:
                try:
                    time.sleep(0.2)
                    cur_timestamp = rospy.Time.now()  # 设置时间戳

                    data = {"observation": {}, "joint_action": None}
                    img_front, img_left, img_right, img_front_depth, img_left_depth, \
                    img_right_depth, puppet_arm_left, puppet_arm_right, robot_base = ros_operator.save_get_frame()
                    data['observation'] = {'head_camera': {'rgb': img_front}, 'left_camera': {'rgb': img_left}, 'right_camera': {'rgb': img_right}}
                    # vis_rgb = data['observation']['head_camera']['rgb']
                    data['joint_action'] = last_action
                    # if last_obs is None:
                    last_obs = data
                    
                except Exception as e:
                    print("Error:", e)
            
            joint_state_msg.position = actions[i][:7]
            master_arm_left_publisher.publish(joint_state_msg)

            joint_state_msg.position = actions[i][7:]
            master_arm_right_publisher.publish(joint_state_msg)

            # joint_state_msg.position = actions[i][:7]
            # puppet_arm_left_publisher.publish(joint_state_msg)

            # joint_state_msg.position = actions[i][7:]
            # puppet_arm_right_publisher.publish(joint_state_msg)

            last_action = actions[i]
            time.sleep(0.15)
            rate.sleep() 

            step_limit -= 1
    
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