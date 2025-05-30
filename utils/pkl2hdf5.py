import h5py, pickle
import numpy as np
import os
import cv2
from collections.abc import Mapping, Sequence
import shutil
from .images_to_video import images_to_video

def images_encoding(imgs):
    encode_data = []
    padded_data = []
    max_len = 0
    for i in range(len(imgs)):
        success, encoded_image = cv2.imencode('.jpg', imgs[i])
        jpeg_data = encoded_image.tobytes()
        encode_data.append(jpeg_data)
        max_len = max(max_len, len(jpeg_data))
    # padding
    for i in range(len(imgs)):
        padded_data.append(encode_data[i].ljust(max_len, b'\0'))
    return encode_data, max_len

def parse_dict_structure(data):
    """
    解析嵌套字典的结构，返回一个解析后的字典。
    如果某个键没有子键（即叶子节点），则创建一个空列表。
    
    Args:
        data: 输入的嵌套字典或 np.array
    
    Returns:
        解析后的字典结构
    """
    if isinstance(data, dict):
        parsed = {}
        for key, value in data.items():
            if isinstance(value, dict):
                parsed[key] = parse_dict_structure(value)
            elif isinstance(value, np.ndarray):
                parsed[key] = []  # 标记为叶子节点
            else:
                parsed[key] = []  # 标记为叶子节点
        return parsed
    else:
        return []  # 如果输入不是字典，则直接返回空列表

def append_data_to_structure(data_structure, data):
    """
    将数据递归地追加到解析后的字典结构中。
    
    Args:
        data_structure: 解析后的字典结构
        data: 要追加的数据
    """
    for key in data_structure:
        if key in data:
            if isinstance(data_structure[key], list):
                # 如果是叶子节点，直接追加数据
                data_structure[key].append(data[key])
            elif isinstance(data_structure[key], dict):
                # 如果是嵌套字典，递归处理
                append_data_to_structure(data_structure[key], data[key])

def load_pkl_file(pkl_path):
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    return data

def create_hdf5_from_dict(hdf5_group, data_dict):
    """
    递归地将嵌套字典结构存储到hdf5文件中。
    
    Args:
        hdf5_group: 当前的hdf5组
        data_dict: 要存储的嵌套字典
    """
    for key, value in data_dict.items():
        if isinstance(value, dict):
            # 如果值是字典，创建一个hdf5组
            subgroup = hdf5_group.create_group(key)
            create_hdf5_from_dict(subgroup, value)
        elif isinstance(value, list):
            # 如果值是numpy数组，直接创建数据集
            value = np.array(value)
            if 'rgb' in key:
                encode_data, max_len = images_encoding(value)
                hdf5_group.create_dataset(key, data=encode_data, dtype=f'S{max_len}')
            else:
                hdf5_group.create_dataset(key, data=value)
        else:
            # 其他类型的数据，尝试存储为字符串
            return
            try:
                hdf5_group.create_dataset(key, data=str(value))
                print('Not np array')
            except Exception as e:
                print(f"Error storing value for key '{key}': {e}")

def pkl_files_to_hdf5_and_video(pkl_files, hdf5_path, video_path):
    data_list = parse_dict_structure(load_pkl_file(pkl_files[0]))
    for pkl_file_path in pkl_files:
        pkl_file = load_pkl_file(pkl_file_path)
        append_data_to_structure(data_list, pkl_file)
    images_to_video(np.array(data_list['observation']['head_camera']['rgb']), out_path=video_path)

    with h5py.File(hdf5_path, 'w') as f:
        create_hdf5_from_dict(f, data_list)

def process_folder_to_hdf5_video(folder_path, hdf5_path, video_path):
    """
    改进的文件列表生成方法，确保严格的数字顺序
    """
    # 获取所有数字命名的pkl文件
    pkl_files = []
    for fname in os.listdir(folder_path):
        if fname.endswith('.pkl') and fname[:-4].isdigit():
            pkl_files.append((int(fname[:-4]), os.path.join(folder_path, fname)))
    
    if not pkl_files:
        raise FileNotFoundError(f"No valid .pkl files found in {folder_path}")
    
    # 按数字排序
    pkl_files.sort()
    pkl_files = [f[1] for f in pkl_files]
    
    # 验证连续性
    expected = 0
    for f in pkl_files:
        num = int(os.path.basename(f)[:-4])
        if num != expected:
            raise ValueError(f"Missing file {expected}.pkl")
        expected += 1
    
    pkl_files_to_hdf5_and_video(pkl_files, hdf5_path, video_path)
    # print(f"Successfully converted {len(pkl_files)} episodes to {hdf5_path}")
