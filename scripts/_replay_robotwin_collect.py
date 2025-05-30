#coding=utf-8
import sys
sys.path.append('./')
import os
import numpy as np
import cv2
import h5py
import argparse
from utils.ros_operator import RosOperator
import rospy

from cv_bridge import CvBridge
import pickle
from std_msgs.msg import Header
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import queue
import time
from utils.pkl2hdf5 import process_folder_to_hdf5_video

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

def main(args):
    ros_operator = RosOperator(args)
    # rospy.init_node("replay_node")

    # puppet_arm_left_publisher = ros_operator.puppet_arm_left_publisher
    # puppet_arm_right_publisher = ros_operator.puppet_arm_right_publisher
    
    master_arm_left_publisher = rospy.Publisher(args.master_arm_left_topic, JointState, queue_size=10)
    master_arm_right_publisher = rospy.Publisher(args.master_arm_right_topic, JointState, queue_size=10)
    
    joint_state_msg = JointState()
    joint_state_msg.header =  Header()
    joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称

    data_path = os.path.join('./RoboTwin/data', f'{args.task_name}', f'{args.setting}', f'episode{args.idx}.hdf5')
    actions = load_robotwin_hdf5_joint(data_path)

    real_action = load_robotwin_hdf5_joint(data_path)

    cache_path = f'./real_data/{args.task_name}/{args.setting}/.cache/episode{args.idx}'
    save_path = f'./real_data/{args.task_name}/{args.setting}/episode{args.idx}.hdf5'
    video_path = f'./real_data/{args.task_name}/{args.setting}/video/episode{args.idx}.mp4'
    traj_data_path = f'./real_data/{args.task_name}/{args.setting}/_traj_data/episode{args.idx}.pkl'

    os.makedirs(cache_path, exist_ok=True)
    os.makedirs(f'./real_data/{args.task_name}/{args.setting}/', exist_ok=True)
    os.makedirs(f'./real_data/{args.task_name}/{args.setting}/video', exist_ok=True)

    rate = rospy.Rate(args.frame_rate)

    actions[:, 6] = actions[:, 6] * 0.88 - 0.22
    actions[:, 13] = actions[:, 13] * 0.88 - 0.22

    # Real Sense
    # cameras = [
    #     RealSenseCam('337322073280', 'right_camera'),
    #     RealSenseCam('337322074191', 'head_camera'),
    #     RealSenseCam('337122072617', 'left_camera')
    # ]

    # for cam in cameras:
    #     cam.start()
 
    # for i in range(20):
    #     print(f'warm up: {i}', end='\r')
    #     for cam in cameras:
    #         color_image = cam.get_latest_image()
    #     time.sleep(0.15)

    i = 0
    while(not rospy.is_shutdown() and i < actions.shape[0]):
        cur_timestamp = rospy.Time.now()  # 设置时间戳
        
        joint_state_msg.header.stamp = cur_timestamp 

        data = {"observation": {}, "joint_action": None}

        img_front, img_left, img_right, img_front_depth, img_left_depth, \
            img_right_depth, puppet_arm_left, puppet_arm_right, robot_base = ros_operator.save_get_frame()

        data['observation'] = {'head_camera': {'rgb': img_front}, 'left_camera': {'rgb': img_left}, 'right_camera': {'rgb': img_right}}

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

        joint_state_msg.position = actions[i][:7]
        master_arm_left_publisher.publish(joint_state_msg)

        joint_state_msg.position = actions[i][7:]
        master_arm_right_publisher.publish(joint_state_msg)

        # joint_state_msg.position = actions[i][:7]
        # puppet_arm_left_publisher.publish(joint_state_msg)

        # joint_state_msg.position = actions[i][7:]
        # puppet_arm_right_publisher.publish(joint_state_msg)

        data['joint_action'] = {'vector': real_action[i], 'left_arm': real_action[i][:6], 'left_gripper': real_action[i][6], "right_arm": real_action[i][7:13], "right_gripper": real_action[i][13]}

        save_to_pkl(data, os.path.join(cache_path, f'{i}.pkl'))
        time.sleep(0.15)
        i += 1
        rate.sleep() 
    
    cv2.destroyAllWindows()
    
    process_folder_to_hdf5_video(cache_path, save_path, video_path)
    print('Finish Merging')

if __name__ == '__main__':
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
    parser.add_argument('--use_depth_image', action='store', type=bool, help='use_depth_image',
                        default=False, required=False)
    
    parser.add_argument('--frame_rate', action='store', type=int, help='frame_rate',
                        default=30, required=False)
    
    parser.add_argument('--only_pub_master', action='store_true', help='only_pub_master',required=False)

    args = parser.parse_args()
    main(args)