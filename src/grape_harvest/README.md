# grape_harvest

设施葡萄园激光采收机器人的 ROS 包。完整说明、演示视频和工程日志在
[仓库主页](../../README.md)。

这里只放包内部的东西。

## 目录

```
urdf/cr10_grape.xacro     机器人：滑移转向平台 + Dobot CR10 + 激光切割头
worlds/vineyard.world     园区世界（由 make_models.py 生成，不要手改）
models/                   trellis 篱架 / dirt_track 土路 / polytunnel 大棚
                          / start_pad 起点标记 —— 全部是生成物
config/
  cr10_grape.srdf         由 gen_srdf.py 生成，不要手改
  vineyard_vins.yaml      由 make_vins_config.py 对着运行中的仿真生成
launch/
  teleop_demo.launch      一键演示：Gazebo + RViz + 遥控 + 自动臂
  grape_pick_demo.launch  自动整垄作业，可录像
  vineyard_gazebo.launch  只起场景和机器人
  moveit.launch           move_group + RViz
scripts/
  make_models.py          场景生成，所有布局常量的唯一来源
  pick_grape.py           采摘循环（--teleop / --row / --check-only / --no-drive）
  drive.py                行间车道保持 + VIO 激励动作
  teleop_base.py          键盘油门，只前后
  world_tf.py             world→odom 修正，VINS-Mono 的占位节点
  stow_check.py           实测收拢构型的臂展包络
  gen_srdf.py             生成 SRDF，stow 位姿由命令行给
  make_vins_config.py     从运行中的仿真生成 VINS-Mono 配置
  run_vins.sh             在容器里跑 VINS-Mono
  record_run.py           录像，收尾时会用真实帧率复核并重编码
  diag.py / diag2.py      工具系朝向、碰撞接触点自检
```

## 生成物不要手改

`worlds/`、`models/`、`config/cr10_grape.srdf`、`config/vineyard_vins.yaml`
都是生成的。改布局改 `make_models.py` 顶部的常量再重新生成：

```bash
python3 scripts/make_models.py          # 场景
python3 scripts/gen_srdf.py config/cr10_grape.srdf --stow 0 -0.9 2.4 -3.0708 -1.5708 0
```

`pick_grape.py` 和 `drive.py` 直接 import `make_models` 的常量，所以场景、
运动规划和底盘控制不会各改各的——这一条是被违反过一次才写进来的：
`make_models.py` 重写之后 `pick_grape.py` 还在 import 已经不存在的名字，
一跑就 ImportError。

## 改臂或改末端之后必须重跑

```bash
rosrun grape_harvest stow_check.py      # 收拢包络会变，SRDF 的 stow 要跟着换
```

CR10 的连杆比 CR5 长 40%，接料围栏又挂在工具下方 0.24 m，两者都进收拢包络。
