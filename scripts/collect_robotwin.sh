#!/bin/bash

task_name=${1}
setting=${2}
st_idx=${3}

while true; do
    if [ "$st_idx" -eq -1 ]; then
        echo "结束脚本运行"
        break
    fi
    
    while true; do
        bash scripts/reset_arm.sh
        bash scripts/preview_robotwin_traj.sh ${task_name} ${setting} ${st_idx}
        
        echo "按下回车继续，输入0重新运行reset_arm.sh和replay_robotwin.sh"
        read input
        if [ "$input" == "0" ]; then
            continue
        else
            break
        fi
    done

    bash scripts/reset_arm.sh
    
    python scripts/_replay_robotwin_collect.py --task_name=${task_name} --setting=${setting} --st_idx=${st_idx}
    echo "reset"
    python scripts/_reset_arm.py 1

    st_idx=$((st_idx + 1))
    echo "当前st_idx为: $st_idx"
    echo "按下回车继续，输入0并回车结束脚本运行"
    read input
    if [ "$input" == "0" ]; then
        st_idx=-1
    fi
done