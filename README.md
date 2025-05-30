<h1 style="text-align:center">AgileX Piper (COBOT-Magic) Deployment</h1>

# 启动ROS、相机等
```
# can使能
bash scripts/enable_can.sh
# 启动roscore
roscore
# 启动相机
bash scripts/start_camera.sh # 如果使用realsense，需要修改该脚本文件
```

# 准备部署
```
# 可视化相机 (optional)
bash scripts/vis_rgb.sh
# 开启部署模式 (策略部署、基于RoboTwin的数据采集)
bash scripts/deploy_mode.sh
```

# 开始数据采集
如果你要使用RoboTwin进行真机自动数据采集，请在此项目根目录运行：`git clone https://github.com/TianxingChen/RoboTwin.git`，并根据`README.md`进行环境配置以及仿真数据预采集

```
# 复位机械臂
bash scripts/reset_arm.sh
# 开启数据采集，数据会存储到`real_data/${task_name}/${setting}`下
bash scripts/collect_robotwin.sh ${task_name} ${setting} ${st_idx}
```

# 连接远程服务器进行推理服务
```
# 连接远程服务器，需要修改对应脚本，默认端口为9403
bash scripts/connect.sh 
# 在远程服务器运行`eval_policy.sh`，开启推理服务，具体算法需要对应修改端口与部署代码，端口需要与上保持一致
bash scripts/eval_policy.sh
# 在本地运行部署代码，这个代码会实现与远程服务器的通信
bash deploy_policy.py
```