# grasp_grape — 葡萄采摘机器人 Gazebo 仿真

ROS Noetic + Gazebo 11 下的葡萄采摘作业仿真：用越疆 **Dobot CR5** 协作臂
夹住果梗、剪断、分格放入采收筐。仿真在 WSL2 的 `Ubuntu-20.04-ROS1` 里开发。

项目分两个阶段，**阶段一已完成并验证，阶段二代码已就位但尚未在仿真里跑过**
（见下方「当前状态」）。

| | 阶段一（已完成） | 阶段二（待验证） |
|---|---|---|
| 场景 | 单个篱架，3 串葡萄 | 4 排 × 每排 4 板，共 30 串随机分布 |
| 底座 | 固定立柱 | 四轮滑移转向移动平台，颠簸土路 |
| 采收筐 | 地面固定 | 随车装在后甲板，6 个格位 |
| 定位 | 世界系固定 | ground truth 发 `world→odom`（VINS-Mono 占位） |

## 演示（阶段一：3/3 全部剪下入筐）

![t2s](media/stills/t2s.png)
![t40s](media/stills/t40s.png)
![t78s](media/stills/t78s.png)
![t118s](media/stills/t118s.png)

完整录像：[`media/cr5_grape_harvest.mp4`](media/cr5_grape_harvest.mp4)
（117 秒，实测 19.44 fps 录制）

阶段一最后一次运行结果：三串全部剪下，等高并排躺在筐底（z 都是 0.422，没有叠罗汉），
碰撞自检确认立柱、铁丝、采收筐、地面均未被碰动。

## 目录结构

```
src/grape_harvest/
  urdf/cr5_grape.xacro      机器人：移动平台 + CR5 + 剪切式末端执行器
  worlds/vineyard.world     园区世界（由 make_models.py 生成）
  models/                   trellis 篱架、dirt_track 土路（生成物）
  config/                   SRDF、MoveIt、Gazebo 控制器、关节限位
  launch/                   grape_pick_demo / vineyard_gazebo / moveit
  scripts/
    make_models.py          场景生成：所有布局常量的唯一来源
    pick_grape.py           采摘循环
    drive.py                行间车道保持
    world_tf.py             world→odom 修正，VINS-Mono 的占位节点
    stow_check.py           实测收拢构型的臂展包络
    diag.py / diag2.py      工具系朝向、碰撞接触点自检
```

## 依赖与搭建

需要 ROS Noetic + Gazebo 11。两个第三方包不 vendor 在本仓库里，请自行 clone 到 `src/`：

```bash
mkdir -p ~/grape_ws/src && cd ~/grape_ws
git clone https://github.com/terrense/grasp_grape.git .

cd src
# 越疆官方 CR5 包，提供 URDF + meshes
git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
# 用来实现"剪断"：运行时建立/解除固连关节
git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git

cd ~/grape_ws && catkin build   # 或 catkin_make
```

## 运行

```bash
# run_sim.sh 已把 source 和 GAZEBO_MODEL_PATH 都包好
~/grape_ws/run_sim.sh roslaunch grape_harvest grape_pick_demo.launch

# 预检：只报 IK 可达性，不动
~/grape_ws/run_sim.sh rosrun grape_harvest pick_grape.py --check-only --no-drive

# 只采当前站位够得着的串，不指挥底盘（把手臂的活和底盘解耦调试）
~/grape_ws/run_sim.sh rosrun grape_harvest pick_grape.py --no-drive --verify

# 整行作业：收拢 → 开到正对某串 → 展臂采下 → 入筐 → 收拢 → 下一串
~/grape_ws/run_sim.sh rosrun grape_harvest pick_grape.py --row 0 --verify

# 行间通过性测试：收臂后开过 1.6 m 的行距
~/grape_ws/run_sim.sh rosrun grape_harvest drive.py _aisle:=0 _stow_first:=true

# 带 MoveIt RViz 界面
~/grape_ws/run_sim.sh roslaunch grape_harvest grape_pick_demo.launch rviz:=true

# 录像（务必配 gui:=false，否则 GUI 渲染抢 CPU 导致相机丢帧）
~/grape_ws/run_sim.sh roslaunch grape_harvest grape_pick_demo.launch \
    gui:=false record:=true video:=$HOME/cr5_grape_harvest.mp4
```

场景要改布局，只改 `scripts/make_models.py` 顶部的常量，然后重新生成：

```bash
python3 src/grape_harvest/scripts/make_models.py
```

`pick_grape.py` 直接 import 这些常量，所以场景和运动规划不会各改各的。

## 当前状态

阶段二的场景、机器人和采摘逻辑都已就位：4 排园区（30 串，果串数量/垂挂深度/大小
随机）、四轮滑移转向平台、floating 底座关节，`drive.py` / `world_tf.py` /
`stow_check.py` 三个新节点，以及移植完成的 `pick_grape.py`。

`pick_grape.py` 的移植内容：果串来源从三个手写 y 坐标改为 `bunch_layout()` 返回的
整块果串列表；采收筐从世界固定点改为 base_link 系的 `CRATE_BASE_XYZ` + `CRATE_SLOTS`
（筐随车走，位姿经 TF 解算）；果串入筐后挂到 `crate_link` 上随车移动；作业循环变成
「收拢 → 开到正对某串 → 展臂 → 采下 → 入筐 → 收拢 → 下一串」。

**每串停一次车，不是每块板停一次。** 行间中线到藤的水平距离恒为 0.80 m，结果铁丝
又在臂基座上方 0.32-0.41 m，正对时三维距离已经 0.87-0.90 m，逼近 CR5 的 0.9 m 臂展
（TCP 再外伸 95 mm）。偏离正对方向 0.2 m 基本就出了包络，所以按 2 m 板距停车会漏掉
大部分果串。可达性判据也必须用三维距离——只看水平会把实际够不到的串放进来。

### 首次整行实测结果（row 0，8 串）

已验证能工作的部分：
- 无头仿真起得来，`move_group` 正常，规划场景 55 个物体（20 立柱 + 4 冠层线 +
  地面 + 30 串果实）。
- 12 个采收筐位姿全部通过碰撞感知 IK——车载筐的 TF 解算和方位角计算是对的。
- 底盘第一次移动正常：1.9 m 用 6 秒，车道误差 0.032 m。
- **完整采摘循环跑通**：正对停车 → 下压 → 夹紧 → 剪断（`stem cut ok=True`）→
  退刀 → 搬运 → 释放入筐（`released into crate ok=True`），第 0、2 串都走完了全程。
- 碰撞自检：立柱、铁丝、地面均未被碰动。

**已修的关键 bug（阶段一遗留）：** 剪断后挂在法兰上的果串碰撞盒被放在了 TCP 的
**上方**而不是下方。`tool_quat` 是 `Rz(az)·Ry(90°+PITCH)`，其 X 列在世界系是
`(-sin·cos, -sin·sin, -cos)`，PITCH=20° 时 z 分量为 −0.94，即 **tool +X 朝下**，
而代码用的是 −X。阶段一所有果梗等长、抓取高度固定，盒子顶端刚好差一点没碰上冠层
铁丝，所以一直没暴露；`bunch_layout()` 把抓取高度随机到 1.16 m 之后，退刀时盒子就
顶进 `canopy_r0`，规划器采不到目标状态，而错误只报成一句 `ABORTED: TIMED_OUT`。

同时修掉的还有两处：`solve_ik` 原先用 `RobotCommander.get_current_state()` 做种子，
那个状态**不含挂在夹爪上的物体**，于是 IK 返回一个规划器随即否决的解；改为从
`/get_planning_scene` 取带 attached objects 的状态。以及剪断之后任何一步失败都会
把果串一直攥在夹爪里，污染后续所有规划——现在失败会先 `abandon()` 放下再返回。

### 仍未解决

1. **底盘会开进葡萄行里。** 第二次移动结束时车停在 `x=+0.703, yaw=56°`
   （`row_x(0)=0.58`，等于骑到藤上了），横向偏差 0.92 m，此后卡死在 y≈-2.77，
   后面 5 次移动全部 180 秒超时，8 串里有 5 串根本没尝试。`drive.py` 是纯车道
   保持器：只按 y 判断到没到，没有避障、没有脱困、偏出车道后无法恢复。这是
   下一步最该修的。
2. **果串留不住在筐里。** 两串都成功释放进筐，但最终位置一个在车侧地面上
   （base_link y=+1.0, z=-0.30），另一个飞到了 70 m 外——后者是物理炸了。
   筐壁很薄且果串只是自由体，平台在土路上一颠就跳出去。合理的做法是释放后用
   link_attacher 把果串固连到 `crate_link`，跟把果串挂上铁丝是同一个手法。
3. **下压段的笛卡尔覆盖率不稳定**，同一行里有 100%、60%、26% 的（26% 那串
   最终失败）。阶段一靠"先解 IK 再关节空间规划"把覆盖率稳到了 100%，移动底座
   之后这个稳定性没能保持。

## 设计说明

机型选择、为什么重写厂商 URDF、"剪断"如何实现、录像帧率为什么要实测、
以及调试过程中踩过的十几个坑（IK 接近轴反向、轨迹容差、RRTConnect 超时……），
都记在 [`src/grape_harvest/README.md`](src/grape_harvest/README.md)。
注意那份文档写于阶段一，尚未覆盖移动平台的内容。

## 已知限制

- `gazebo_ros_control` 没配 PID 增益，走 `Joint::SetPosition` 直接定位，关节相当于无限刚性，
  **不能用来评估力控、碰撞载荷或跟踪精度**。
- 冠层叶片只有 visual 没有 collision，相当于假设采收前已完成疏叶。
- 采收筐只有 6 个格位，一次作业最多 6 串，装满即停——没有建模卸筐/换筐。
