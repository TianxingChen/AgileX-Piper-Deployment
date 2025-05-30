import sys
import subprocess
sys.path.append('./') 
sys.path.append(f'./policy')
sys.path.append('./description/utils') 
# from envs import CONFIGS_PATH

import os
import numpy as np
from pathlib import Path
from collections import deque
import traceback

import yaml
from datetime import datetime
import importlib
import argparse
import pdb
import socket
import pickle

from generate_episode_instructions import *

current_file_path = os.path.abspath(__file__)
parent_directory = os.path.dirname(current_file_path)

def class_decorator(task_name):
    envs_module = importlib.import_module(f'envs.{task_name}')
    try:
        env_class = getattr(envs_module, task_name)
        env_instance = env_class()
    except:
        raise SystemExit("No Task")
    return env_instance

def eval_function_decorator(policy_name, model_name):
    try:
        policy_model = importlib.import_module(policy_name)
        func = getattr(policy_model, model_name)
        return func
    except ImportError as e:
        raise e

def get_camera_config(camera_type):
    camera_config_path = os.path.join(parent_directory, '../task_config/_camera_config.yml')

    assert os.path.isfile(camera_config_path), "task config file is missing"

    with open(camera_config_path, 'r', encoding='utf-8') as f:
        args = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert camera_type in args, f'camera {camera_type} is not defined'
    return args[camera_type]

def get_embodiment_config(robot_file):
    robot_config_file = os.path.join(robot_file, 'config.yml')
    with open(robot_config_file, 'r', encoding='utf-8') as f:
        embodiment_args = yaml.load(f.read(), Loader=yaml.FullLoader)
    return embodiment_args

def main(usr_args):
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    task_name = usr_args['task_name']
    setting = usr_args['setting']
    # checkpoint_num = usr_args['checkpoint_num']
    policy_name = usr_args['policy_name']
    instruction_type = usr_args['instruction_type']
    save_dir = None
    video_save_dir = None
    video_size = None

    get_model = eval_function_decorator(policy_name, 'get_model')

    usr_args['left_arm_dim'] = 6
    usr_args['right_arm_dim'] = 6
    
    seed = usr_args['seed']

    model = get_model(usr_args)
    eval_policy(policy_name, model)
    

def eval_policy(policy_name, model, host='0.0.0.0', port=9403):
    eval_func = eval_function_decorator(policy_name, 'eval')
    reset_func = eval_function_decorator(policy_name, 'reset_model')
    update_obs_func = eval_function_decorator(policy_name, 'update_obs')

    reset_func(model)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 防止地址占用
        s.bind((host, port))
        s.listen()
        print(f"Server listening on {host}:{port}")
        
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                
                # 接收数据
                data = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:  # 连接关闭
                        break
                    data += chunk
                    # print(f"Received chunk of size: {len(chunk)}")
                
                if not data:
                    print("Received empty data")
                    continue
                
                # 反序列化对象
                try:
                    observation_list = pickle.loads(data)
                    observation = observation_list['current']
                    last_observation = observation_list['last']
                    if observation_list['flag']:
                        reset_func(model)
                    print("Object received successfully")
                    
                    update_obs_func(model, last_observation)
                    actions = eval_func(model, observation)
                    
                    # 发送结果
                    result_data = pickle.dumps(actions)
                    conn.sendall(result_data)
                    print("Result sent back to client")
                    
                except Exception as e:
                    print(f"Error processing object: {e}")
                    error_data = pickle.dumps({"error": str(e)})
                    conn.sendall(error_data)

def parse_args_and_config():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, required=True)
    parser.add_argument('--overrides', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    with open(args.config, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # Parse overrides
    def parse_override_pairs(pairs):
        override_dict = {}
        for i in range(0, len(pairs), 2):
            key = pairs[i].lstrip('--')
            value = pairs[i+1]
            try:
                value = eval(value)
            except:
                pass
            override_dict[key] = value
        return override_dict

    if args.overrides:
        overrides = parse_override_pairs(args.overrides)
        config.update(overrides)

    return config

if __name__ == "__main__":
    usr_args = parse_args_and_config()    

    main(usr_args)