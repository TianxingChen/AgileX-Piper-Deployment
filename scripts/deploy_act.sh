data_path=~/Desktop/data
# task_name=collect_bowl_aloha_hdf5
task_name=collect_bowl
ckpt_dir=~/Desktop/ckpt/$task_name
# ckpt_name=policy_epoch_2100_seed_0.ckpt
ckpt_name=policy_epoch_3400_seed_0.ckpt
# ckpt_name=policy_epoch_4100_seed_0.ckpt

cd ~/cobot_magic/aloha-devel
python act/inference.py --ckpt_dir $ckpt_dir --task_name $task_name --ckpt_name $ckpt_name