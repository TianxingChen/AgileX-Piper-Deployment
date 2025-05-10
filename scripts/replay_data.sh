task_name=${1}
idx=${2}
data_path=~/Desktop/AgileX-Piper-Deployment/real_data

cd ~/cobot_magic/collect_data
python replay_data.py --dataset_dir $data_path --task_name $task_name --episode_idx $idx
