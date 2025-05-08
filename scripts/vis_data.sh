idx=${1}
data_path=~/Desktop/data

cd ~/cobot_magic/collect_data
python visualize_episodes.py --dataset_dir $data_path --task_name aloha_mobile_dummy --episode_idx $idx