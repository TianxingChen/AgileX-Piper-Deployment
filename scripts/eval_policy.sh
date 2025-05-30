#!/bin/bash

policy_name=RDT
task_name=${1}
model_name=${2}
checkpoint_id=${3}
seed=0
gpu_id=${4}

DEBUG=False
export CUDA_VISIBLE_DEVICES=${gpu_id}
echo -e "\033[33mgpu id (to use): ${gpu_id}\033[0m"

cd # move to root

python script/eval_policy_server.py --config RoboTwin/policy/$policy_name/deploy_policy.yml \
    --overrides \
    --task_name ${task_name} \
    --setting ${model_name} \
    --seed ${seed} \
    --checkpoint_id ${checkpoint_id} \
    --policy_name ${policy_name}
