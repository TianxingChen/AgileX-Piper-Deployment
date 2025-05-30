task_name=${1}
idx=${2}
data_path=/home/agilex/Desktop/RoboTwin_Close/data/bottle_pick_real/piper+piper-m1_b1_l1_h0_c0_D435

cd ~/cobot_magic/collect_data
python replay_data.py --dataset_dir $data_path --task_name $task_name --episode_idx $idx
