#coding=utf-8
import os
import numpy as np
import cv2
import h5py
import argparse
import rospy

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist

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
    
    
    i = 0
    while(not rospy.is_shutdown() and i < 30):

        cur_timestamp = rospy.Time.now()  # 设置时间戳
        
        joint_state_msg.header.stamp = cur_timestamp 

        joint_state_msg.position =  [0, 0, 0, 0, 0, 0, 0]
        master_arm_left_publisher.publish(joint_state_msg)

        joint_state_msg.position = [0, 0, 0, 0, 0, 0, 0]
        master_arm_right_publisher.publish(joint_state_msg)

        joint_state_msg.position = [0, 0, 0, 0, 0, 0, 0]
        puppet_arm_left_publisher.publish(joint_state_msg)

        joint_state_msg.position = [0, 0, 0, 0, 0, 0, 0]
        puppet_arm_right_publisher.publish(joint_state_msg)

        i += 1
        rate.sleep() 
    
    joint_state_msg.position =  [0, 0, 0, 0, 0, 0, 0.66]
    master_arm_left_publisher.publish(joint_state_msg)

    joint_state_msg.position = [0, 0, 0, 0, 0, 0, 0.66]
    master_arm_right_publisher.publish(joint_state_msg)

    joint_state_msg.position = [0, 0, 0, 0, 0, 0, 0.66]
    puppet_arm_left_publisher.publish(joint_state_msg)

    joint_state_msg.position = [0, 0, 0, 0, 0, 0, 0.66]
    puppet_arm_right_publisher.publish(joint_state_msg)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_name', action='store', type=str, help='Task name.',
                        default="aloha_mobile_dummy", required=False)

    parser.add_argument('--episode_idx', action='store', type=int, help='Episode index.',default=0, required=False)
    
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
    # python collect_data.py --max_timesteps 500 --is_compress --episode_idx 0 