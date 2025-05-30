sleep_flag=${1}

# 如果 sleep_flag 为空，则设置为 false，否则为 true
if [ -z "$sleep_flag" ]; then
    sleep_flag=false
else
    sleep_flag=true
fi

python scripts/_reset_arm.py --sleep=$sleep_flag