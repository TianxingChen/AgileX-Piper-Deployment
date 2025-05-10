from utils.pkl2hdf5 import process_folder_to_hdf5_video


for i in range(20):
    process_folder_to_hdf5_video(f'/home/agilex/Desktop/AgileX-Piper-Deployment/real_data/robotwin_reflash/episode{i}', f'/home/agilex/Desktop/AgileX-Piper-Deployment/real_data/robotwin_hdft/dual_bottles_pick_easy/episode{i}.hdf5', f'/home/agilex/Desktop/AgileX-Piper-Deployment/real_data/robotwin_hdft/dual_bottles_pick_easy/video/episode{i}.mp4')