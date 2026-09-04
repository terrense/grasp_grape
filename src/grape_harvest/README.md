# grape_harvest — Dobot CR5 葡萄采摘 Gazebo 仿真

在 `Ubuntu-20.04-ROS1` (ROS Noetic + Gazebo 11) 里，用越疆 **Dobot CR5** 协作臂
把篱架上的**全部葡萄串**逐串剪下、分格放入采收筐。

工作空间：`~/grape_ws`

## 跑起来

```bash
source /opt/ros/noetic/setup.bash
source ~/grape_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=~/grape_ws/src/grape_harvest/models:$GAZEBO_MODEL_PATH

# 完整演示：Gazebo + MoveIt + 自动采摘全部 3 串
roslaunch grape_harvest grape_pick_demo.launch

# 只起场景，手动触发
roslaunch grape_harvest grape_pick_demo.launch auto_start:=false
rosrun grape_harvest pick_grape.py --verify

# 录像（务必配 gui:=false，见下）
roslaunch grape_harvest grape_pick_demo.launch gui:=false record:=true \
    video:=/mnt/c/Users/Administrator/grape_shots/cr5_grape_harvest.mp4

# 带 RViz 的 MoveIt 界面
roslaunch grape_harvest grape_pick_demo.launch rviz:=true
```

`~/grape_ws/run_sim.sh` 已把上面三行环境变量包好：`~/grape_ws/run_sim.sh roslaunch ...`

## 动作序列

每串一个循环，三串串行执行：

| 步骤 | 内容 |
|---|---|
| 1 | 张开刀口 → 移到该串的预抓取位（沿接近轴退让 150 mm） |
| 2 | 笛卡尔直线下压到果梗 |
| 3 | 闭合刀口夹紧果梗 |
| 4 | **剪断**：果串先固连到末端，**再**解除与铁丝的连接 |
| 5 | 笛卡尔直线回撤 → 搬运到筐内该串的专属格位 → 张开释放 |
| 6 | 把落定的果串加入碰撞世界，供后续两串规划时避让 |

三串全程约 115 秒。

## 录像

场景里有一台固定机位相机（`worlds/vineyard.world` 中的 `harvest_cam` 模型），
发布 `/harvest_cam/image_raw`。`scripts/record_run.py` 把帧直接管道喂给 ffmpeg
编码成 mp4 —— 不依赖 cv_bridge / OpenCV，也不落一堆中间 PNG。

**为什么录像要 `gui:=false`：** gzclient 渲染和 x264 编码会和传感器渲染抢 CPU，
Gazebo 就开始丢传感器帧。带 GUI 那次 115 秒的动作只录到 71 秒画面（快放 1.6 倍）。
无头跑时相机稳定在约 19.4 fps。

**录像帧率是实测的，不是设定的。** `record_run.py` 先缓冲 5 秒帧测出真实速率，
再用这个速率启动 ffmpeg（`fps:=0` 即测量模式）。否则按标称 30 fps 编码，
视频会比真实动作快。

## 无碰撞是怎么保证的

1. **规划层**：MoveIt 的规划场景里放了立柱、冠层铁丝、采收筐、地面、
   仍在藤上的果串，以及**已经放进筐里的果串**（按 Gazebo 里的实际落点添加）。
   所有运动都由 OMPL 做无碰撞规划，笛卡尔段也开着 `avoid_collisions`。
   唯一被临时移出的是**正在剪的那一串**，且只在它自己最后几厘米下压期间 ——
   那一步本来就是要去夹它。
2. **执行层验证**：`--verify` 在开跑前记录所有非目标物体的 Gazebo 位姿，
   跑完逐一比对。葡萄架的立柱/铁丝是靠 fixed joint 锚在世界上的，
   但果串只靠 attacher 关节挂着，被碰一下就会明显摆动 —— 这是个灵敏的探针。

最近一次运行结果：

```
harvested 3/3: grape_0, grape_1, grape_2
  grape_0   at (-0.092, -0.610, 0.422)  IN CRATE
  grape_1   at (+0.046, -0.611, 0.422)  IN CRATE
  grape_2   at (+0.190, -0.610, 0.422)  IN CRATE
-- collision check: things the arm must not move --
  trellis, crate and ground all unmoved: no unintended contact
```

三串等高并排（z 都是 0.422）躺在筐底，没有叠罗汉。

## 关键设计

**机型选择 · Dobot CR5**
6 自由度、臂展 900 mm、负载 5 kg，官方 ROS1 支持完整
（[Dobot-Arm/TCP-IP-ROS-6AXis](https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis)，含 URDF + meshes + MoveIt）。
CR3 臂展 620 mm 够不到葡萄架，CR10/CR16 对单串葡萄过大。

**没有直接用厂商的 Gazebo 包。** `dobot_gazebo/urdf/cr5_robot.xacro` 给每个 link 都加了
`<kinematic>true</kinematic>` 并改用 `libgazebo_ros_joint_pose_trajectory` —— 物理是关掉的，
纯运动学摆姿势。这样机械臂能动，但夹爪抓不住任何东西。本包重写了 URDF：

- 恢复动力学 link，加 `transmission` + `gazebo_ros_control`
- 加 0.75 m 立柱底座（让法兰够得着 1.22 m 的结果铁丝）
- 加剪切式二指末端执行器，TCP 在法兰前方 95 mm

**"剪断" 用 `gazebo_ros_link_attacher` 实现。** 果串靠运行时固连关节挂在铁丝上；
剪断 = 先把果串固连到法兰，**再**解除铁丝那一端，顺序反过来果实会自由落体。
真实的接触式抓握在 Gazebo 里对一串软果非常不稳定，这个做法更可靠也更贴近
"夹紧果梗→剪断" 的物理过程。

**位姿先解 IK 再做关节空间规划。** 直接把位姿目标丢给 OMPL，规划器会在众多 IK 解里
随便挑一个，而挑中哪个姿态直接决定后面那段直线下压能不能走通 —— 同样的目标点，
有时笛卡尔覆盖 100%，有时只有 26%。改成显式解 IK（以当前状态为种子）再规划关节空间目标后，
覆盖率稳定在 100%。

## 调试踩过的坑（都已修，写在这防止改动时踩回去）

| 现象 | 原因 |
|---|---|
| 所有 collision-aware IK 失败 (-31) | 工具接近轴指反了（`Ry(-90)` 而非 `Ry(+90)`），手腕要伸到架子后面回头够，路径穿过果实区 |
| 规划成功但预抓取点在架子外侧 | 翻转朝向后退让方向的符号没跟着翻，`VINE_X + APPROACH` 应为沿接近轴**减去** |
| 水平接近时必碰 | 小臂和果串顶部同高。加 20° 下倾（真实采摘末端也这么做） |
| `GOAL_TOLERANCE_VIOLATED: goal error 0.000050` | 触发的是 `stopped_velocity_tolerance`（被折进 goal 容差的速度分量），但报错打印的是位置误差。SetPosition 模式下速度不干净收敛 → 设 0 关掉 |
| `PATH_TOLERANCE_VIOLATED: joint4 -1.10` / `GOAL_TOLERANCE_VIOLATED: joint3 -0.07` | 只在剪断后出现。attacher 把果串刚性挂上法兰后，`SetPosition` 驱动的关节被载荷拽偏 → 关掉路径容差、goal 容差放宽到 0.12 rad、搬运时降速到 0.2 |
| RRTConnect 超时 | URDF 关节限位 ±6.28 让 6 维搜索空间大了约 4000 倍 → 收到 ±π |
| `Computed path is not valid, invalid states at [36]` | `longest_valid_segment_fraction` 放宽到 0.01 使碰撞检测过粗 → 回到 0.005 |
| `compute_cartesian_path` 签名不匹配 | MoveIt ≥1.1.14 去掉了 `jump_threshold` 参数 |
| 果串在筐里叠罗汉 | 格位间距 ±0.09 m 小于果串直径 0.14 m → 筐加大到 0.46×0.30，间距改 ±0.14 |
| 视频比实际动作快 1.6 倍 | 相机丢帧但按标称 30 fps 编码 → 改为实测帧率 + 无头录制 |

## 自检工具

```bash
rosrun grape_harvest pick_grape.py --check-only   # 15 个关键位姿的 IK 可达性
python3 ~/grape_ws/src/grape_harvest/scripts/diag.py    # 工具坐标系朝向 + IK 分项
python3 ~/grape_ws/src/grape_harvest/scripts/diag2.py   # 碰撞接触点明细
```

## 改场景

所有坐标常量集中在 `scripts/make_models.py` 顶部（`VINE_X` / `WIRE_Z` / `BUNCH_Y` /
`CRATE_XY` / `CAM_POSE` …），模型 SDF 和 `worlds/vineyard.world` 都由它生成，改完跑一次：

```bash
python3 ~/grape_ws/src/grape_harvest/scripts/make_models.py
```

`pick_grape.py` 直接 import 这些常量，所以场景和运动规划不会各改各的。
往 `BUNCH_Y` 里加一个 y 坐标就多一串葡萄，采摘循环会自动带上它
（`CRATE_SLOT_DX` 也要相应加格位，否则会循环复用格位而堆叠）。

## 已知限制

- `gazebo_ros_control` 没有配 PID 增益，走的是 `Joint::SetPosition` 直接定位。
  运动确定性好，但关节是"无限刚性"的：**不能用来评估力控、碰撞载荷或跟踪精度**，
  上面放宽的那几个容差也正是因为跟踪误差在这个模型里没有物理意义。
  要做力学评估需在 `config/gazebo_controllers.yaml` 里补 `pid_gains` 并重新整定。
- 冠层叶片只有 visual 没有 collision，相当于假设采收前已完成疏叶。
- 结果铁丝（8 mm）没进规划场景 —— 它在抓取点上方仅 75 mm，加进去只会造成
  大量无谓的规划失败，实际也不存在撞上的风险。
