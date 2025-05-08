task_name=${1}
setting=${2}
idx=${3}

bash scripts/reset_arm.sh
bash scripts/replay_robotwin.sh ${task_name} ${setting} ${idx}
bash scripts/reset_arm.sh
python scripts/_collect_real_world_robotwin.py --task_name=${task_name} --setting=${setting} --idx=${idx}
bash scripts/reset_arm.sh