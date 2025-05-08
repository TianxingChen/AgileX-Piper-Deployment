task_name=${1}
idx=${2}

max_time_steps=250
dataset_prefix=$AGILEX_DATA_ROOT/real_data
cd ~/cobot_magic/collect_data
python collect_data.py --dataset_dir=$dataset_prefix --task_name=$task_name --max_timesteps=$max_time_steps --episode_idx $idx
