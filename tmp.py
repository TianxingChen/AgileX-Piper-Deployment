import os, pdb
import pickle


all_path = '/home/agilex/Desktop/AgileX-Piper-Deployment/real_data/robotwin/dual_bottles_pick_easy'

for j in range(20):
    path = os.path.join(all_path, f'episode{j}')
    # 初始化计数器
    pkl_count = 0

    # 遍历当前目录中的所有文件和文件夹
    for filename in os.listdir(path):
        # 检查文件扩展名是否为 .pkl
        if filename.endswith('.pkl'):
            pkl_count += 1
    for i in range(pkl_count):

        # 加载 .pkl 文件
        file_path = os.path.join(path, f'{i}.pkl')
        with open(file_path, 'rb') as f:
            data = pickle.load(f)
        
        new_data = {}
        new_data['observation'] = {'head_camera': {"rgb": data['images']['cam_high']}, 'left_camera': {"rgb": data['images']['cam_left_wrist']}, 'right_camera': {"rgb": data['images']['cam_right_wrist']}, }
        new_data['joint_action'] = {'vector': data['qpos'], 'left_arm': data['qpos'][:6], 'left_gripper': data['qpos'][6], 'right_arm': data['qpos'][7:13], 'right_gripper': data['qpos'][13]}
        # pdb.set_trace()

        # 指定保存的文件路径
        file_path = f'/home/agilex/Desktop/AgileX-Piper-Deployment/real_data/robotwin_reflash/episode{j}/{i}.pkl'
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        # 使用 pickle 将对象保存到文件
        with open(file_path, 'wb') as f:
            pickle.dump(new_data, f)
        