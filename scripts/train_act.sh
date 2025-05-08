data_num=${1}
data_path=~/Desktop/data
task_name=collect_bowl_aloha_hdf5
ckpt_dir=~/Desktop/ckpt/$task_name

mkdir $ckpt_dir

cd ~/cobot_magic/aloha-devel
python act/train.py --dataset_dir $data_path --ckpt_dir $ckpt_dir --batch_size 2 --num_epochs 5000 --num_episodes $data_num --task_name $task_name
