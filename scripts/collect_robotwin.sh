#!/bin/bash

task_name=dual_bottles_pick_easy
setting=piper+piper-m0_b0_l0_h0_c0_D435
idx=${1}

while true; do
    if [ "$idx" -eq -1 ]; then
        echo "结束脚本运行"
        break
    fi
    
    while true; do
        bash scripts/reset_arm.sh
        bash scripts/replay_robotwin.sh ${task_name} ${setting} ${idx}
        
        echo "按下回车继续，输入0重新运行reset_arm.sh和replay_robotwin.sh"
        read input
        if [ "$input" == "0" ]; then
            continue
        else
            break
        fi
    done

    bash scripts/reset_arm.sh
    python scripts/_collect_real_world_robotwin.py --task_name=${task_name} --setting=${setting} --idx=${idx}
    python scripts/_reset_arm_sleep.py

    idx=$((idx + 1))
    echo "当前idx为: $idx"
    echo "按下回车继续，输入0并回车结束脚本运行"
    read input
    if [ "$input" == "0" ]; then
        idx=-1
    fi
done