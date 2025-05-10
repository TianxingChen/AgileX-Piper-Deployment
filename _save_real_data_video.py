
import os
import pickle
import pdb

def load_pkl(file_path):
    with open(file_path, 'rb') as f:
        data = pickle.load(f)
    return data

def ensure_dir(file_path):
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory)

def create_video(image_folder, output_video_file, save_dir='./task_video/', fps=120):

    video_path = f'{save_dir}/{output_video_file}'
    ensure_dir(video_path)
    # 获取并排序图像文件
    pkl_files = [x for x in os.listdir(image_folder) if x.endswith(".pkl")]
    pkl_files.sort(key=lambda x: int(x.split('.')[0]))

    import subprocess
    ffmpeg = subprocess.Popen([
        'ffmpeg', '-y', '-loglevel', 'error',
        '-f', 'rawvideo',
        '-pixel_format', 'rgb24',
        '-video_size', '640x480',
        '-framerate', '10',
        '-i', '-',
        '-pix_fmt', 'yuv420p',
        '-vcodec', 'libx264',
        '-crf', '23',
        video_path
    ], stdin=subprocess.PIPE)

    # 添加图像到视频
    for i, pkl_file in enumerate(pkl_files, start=1):
        pkl_path = os.path.join(image_folder, pkl_file)
        data = load_pkl(pkl_path)
        # pdb.set_trace()
        # img = data['images']['cam_high']
        # img = data['images']['cam_left_wrist']
        img = data['images']['cam_right_wrist']
        ffmpeg.stdin.write(img.tobytes())

    ffmpeg.stdin.close()
    ffmpeg.wait()
    del ffmpeg 
    print("\nVideo creation complete.")

if __name__ == '__main__':
    file_dir = './real_data/robotwin'
    task_name = input()
    save_dir = './real_data_video'
    episode_num = 3
    for i in range(episode_num):
        create_video(f'{file_dir}/{task_name}/episode{i}/',f'{task_name}/episode{i}.mp4', save_dir=save_dir)