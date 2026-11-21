---
title: "RK3588 上打通 MID-360S + FAST-LIO2 + MAVROS + EGO-Planner：全链路实战与踩坑记录"
date: 2026-11-19 10:00:00
updated: 2026-11-19 10:00:00
tags:
  - ROS2
  - RK3588
  - FAST-LIO2
  - EGO-Planner
  - MAVROS
  - 无人机
categories:
  - 机器人实战
---

> 本文基于 RK3588 板卡上的实际工程整理而成，所有配置、命令与参数均来自板端真实文件。

---

## 摘要

目标：让一架 PX4 飞控的无人机，只靠一颗 Livox MID-360S 激光雷达，在**没有 GPS、没有外部动捕**的环境下完成「定位 → 建图 → 自主避障规划 → 飞控执行」的完整闭环。

最终跑通的链路：

```
MID-360S ──> /livox/lidar + /livox/imu
              │
              ▼
        FAST-LIO2  ──> /Odometry  +  /cloud_registered
              │              │
              │              ├──> external_odom_to_mavros ──> /uav1/vision_pose/pose ──> PX4（定位回灌）
              │              │
              │              └──> EGO-Planner ──> /drone_0_planning/pos_cmd
              │                                          │
              │                                          ▼
              │                              ego_to_mavros pos_cmd_bridge
              │                                          │
              │                                          ▼
              │                              /uav1/setpoint_raw/local ──> PX4（OFFBOARD 控制）
              ▼
         /cloud_registered ──> UDP 转发 ──> 地面站/图传显示
```

---

## 1. 环境与硬件清单

| 项目     | 型号 / 版本                                         |
| ------ | ----------------------------------------------- |
| 开发板    | RK3588（aarch64，4 核 Cortex-A76 + 4 核 Cortex-A55） |
| 系统     | Ubuntu 22.04.5 LTS，内核 6.1.118                   |
| ROS    | ROS2 Humble                                     |
| 激光雷达   | Livox MID-360S（非重复扫描，10Hz）                      |
| 飞控     | PX4，通过 UDP 与板卡通信                                |
| 相机（可选） | RealSense D435i（仅 FAST-LIVO2 视觉方案需要）            |
| DDS    | `rmw_cyclonedds_cpp`（**全链路统一，不可混用**）            |

---

## 2. 网络规划：

这套系统同时存在**四个网段**，物理上混在几个网口上，必须先画清楚再动手：

| 用途               | 网段 / IP                               | 所在接口               | 说明                                           |
| ---------------- | ------------------------------------- | ------------------ | -------------------------------------------- |
| 雷达数据（FAST-LIO2）  | 主机 `192.168.1.5`，雷达配置 `192.168.1.174` | `eth_lidar`        | 驱动 bind 配置里的 `host_ip`                       |
| 雷达数据（FAST-LIVO2） | 主机 `192.168.1.50`                     | `eth_lidar`        | 同一颗雷达，两套算法要求不同主机 IP                          |
| 板卡保留地址           | `192.168.111.204`                     | `eth_lidar`        | 切 IP 时保留，避免影响其他网段                            |
| 飞控通信             | 飞控 `10.41.10.2:14550`                 | 板卡网口               | MAVROS 通过 `udp://:14550@10.41.10.2:14550` 连接 |
| 地面站/遥控链路         | 对端 `192.168.144.11`                   | `eth0` / `eth_usb` | 点云 UDP 8888，控制指令 UDP 9999                    |

### 2.1 两套算法抢同一个网口怎么办

FAST-LIO2 要求网卡 IP 是 `192.168.1.5`，FAST-LIVO2 要求 `192.168.1.50`。驱动会 bind 配置 JSON 里的 `host_ip`，**网卡地址和它不一致就收不到任何点云**，而且不会报错，只是话题没数据——排查起来非常痛苦。

解决方案是写一个切换脚本 `switch_lidar_ip.sh`，用 `nmcli` 原子地改地址并保留其它网段：

```bash
# 切到 FAST-LIVO2
~/switch_lidar_ip.sh 192.168.1.50
# 切回 FAST-LIO2
~/switch_lidar_ip.sh 192.168.1.5
# 只看现状不改
~/switch_lidar_ip.sh
```

脚本的核心逻辑：

```bash
NIC=eth_lidar
KEEP=192.168.111.204          # 同网卡上要保留的其它网段
CN=$(nmcli -t -f NAME,DEVICE connection show | awk -F: -v d=$NIC '$2==d{print $1; exit}')
ADDRS="$WANT/24,$KEEP/24"
sudo nmcli connection modify "$CN" ipv4.addresses "$ADDRS"
sudo nmcli connection down "$CN" && sleep 1
sudo nmcli connection up   "$CN" && sleep 2
```

FAST-LIVO2 的一键脚本更进一步，把「改 IP → 启动 → 退出时还原 IP」做成了闭环，并提供 `net` / `netrestore` 子命令用于只读检查和回滚。

> 坑点提示：一键启动脚本顶部注释写「雷达 IP: 192.168.1.191」，而实际 `MID360s_config.json` 里的雷达 IP 是 `192.168.1.174`。注释与配置不一致时，**以驱动实际加载的 JSON 为准**，否则 ping 半天 ping 错对象。

---

## 3. 工作区结构

不要把几十个功能塞进一个 workspace。板卡上最终形成了这些相对独立的工作区：

| 工作区                    | 作用                                                            |
| ---------------------- | ------------------------------------------------------------- |
| `~/mid360_ws`          | `livox_ros_driver2` 雷达驱动（FAST-LIO2 用）                         |
| `~/mid360_fastlio_ws`  | FAST_LIO_ROS2 建图定位 + 全部一键启动脚本 + 自定义 MAVROS/EGO launch         |
| `~/fastlivo2_isolated` | FAST-LIVO2（激光+视觉紧耦合）**完全隔离**的一套环境                             |
| `~/mavros_ws`          | MAVROS                                                        |
| `~/ros2_ws`            | `ego-planner-swarm`、`ego_to_mavros`、`external_odom_to_mavros` |
| `~/d435i_ws`           | RealSense 相机驱动                                                |

`fastlivo2_isolated` 的设计很关键：它有自己的 `setup_env.sh`（设定 `ROS_DOMAIN_ID=42`，只 source 自己的两个 install），启动脚本明确声明「与旧脚本完全独立」。

---

## 4. 第一步：雷达驱动 livox_ros_driver2

```bash
source /opt/ros/humble/setup.bash
source ~/mid360_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

发布话题：`/livox/lidar`（PointCloud2）、`/livox/imu`（Imu）。

验收：

```bash
ros2 topic hz /livox/lidar     # 期望 ~10Hz
ros2 topic hz /livox/imu       # 期望 100Hz（配置里 imu_publish_freq=100.0）
```

配置文件 `MID360s_config.json` 关键点：

```json
{
  "Mid360s": {
    "host_net_info": [{ "host_ip": "192.168.1.50", ... }]
  },
  "lidar_configs": [{
    "ip": "192.168.1.174",
    "lidar_type": 8,
    "imu_publish_freq": 100.0,
    "imu_enable": true
  }]
}
```

雷达刚上电要预热：FAST-LIVO2 脚本里专门留了 `PREHEAT=10` 秒倒计时，期间**必须保持机体静止**，否则 IMU 零偏没收敛，后面建图会飘。

RViz 里 `/livox/lidar` 的原始点云（按距离着色，可见 MID-360S 非重复扫描的密集线束）：

![MID-360S 雷达原始点云（RViz）](/images/lidar-rviz-pointcloud.jpg)

---

## 5. 第二步：FAST-LIO2 建图定位

```bash
source /opt/ros/humble/setup.bash
source ~/mid360_fastlio_ws/install/setup.bash
source ~/mid360_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml rviz:=false
```

发布话题：`/Odometry`（nav_msgs/Odometry）、`/cloud_registered`（注册后的全局点云）。

`config/mid360.yaml` 关键参数：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/livox/imu"
    time_sync_en: false          # 有硬件同步才开；没把握就别动
    time_offset_lidar_to_imu: 0.0
preprocess:
    lidar_type: 4                # 4 = 任意标准点云输入
    scan_line: 4
    blind: 0.5
    timestamp_unit: 3
    scan_rate: 10
mapping:
    extrinsic_est_en: true       # 在线估计 IMU-LiDAR 外参
    extrinsic_T: [ -0.011, -0.02329, 0.04412 ]
publish:
    scan_publish_en: true
    dense_publish_en: true
pcd_save:
    pcd_save_en: true
    interval: -1                 # -1 = 全部帧存成一个 pcd
```

注意这里 `lidar_type: 4`，而 FAST-LIVO2 的 `avia.yaml` 里是 `lidar_type: 1`——一个吃标准点云、一个吃 Livox 原生点云，**参数不能互相抄**。

外参 `extrinsic_T / extrinsic_R` 和时间偏移 `time_offset_lidar_to_imu` 理论上应由 LI-Init 之类的算法离线标定，再填回 YAML。没标准时先填 0，能跑起来，但精度会差一截。

验收：

```bash
ros2 topic hz /Odometry            # ~10Hz
ros2 topic echo /Odometry --once   # 看 position 是否在动
```

FAST-LIO2 输出的全局点云地图 `/cloud_registered`（CloudCompare 中查看，按高度着色，可见建图后的场景结构）：

![FAST-LIO2 建图效果（CloudCompare）](/images/fastlio-mapping-result.jpg)

---

## 6. 第三步：把定位结果回灌飞控

PX4 支持通过 MAVROS 的 `vision_pose` 接口接收外部定位（VIO）。板卡上写了一个通用桥接包 `external_odom_to_mavros`：

```bash
ros2 launch external_odom_to_mavros bridge.launch.py \
  input_type:=odometry \
  input_topic:=/Odometry \
  output_topic:=/uav1/vision_pose/pose \
  output_frame_id:=map \
  publish_rate:=30.0
```

节点逻辑很朴素：订阅 `nav_msgs/Odometry`，把 `pose.pose` 拆成 `PoseStamped`，用定时器以 30Hz 重发布。真正有价值的是两个细节：

1. **时间戳默认用当前时间，不用输入时间戳**（`use_input_stamp: false`）。External localization 用旧时间戳容易被 PX4 判定为超时数据。
2. **过期保护**：`max_input_age: 0.2`，输入超过 200ms 不再转发并打 warning，避免用陈旧位姿"骗"飞控。

验收：

```bash
ros2 topic hz /uav1/vision_pose/pose   # 期望 >20Hz
```

---

## 7. 第四步：MAVROS 接入飞控（本文最大的坑）

一开始用官方 launch：

```bash
ros2 launch mavros px4.launch fcu_url:=udp://:14550@10.41.10.2:14550 namespace:=uav1
```

然后撞上两个连环大坑。

### 坑 1：进程 exit code -6，飞控永远连不上

**现象**：MAVROS 一启动就崩，`ros2 node list` 里什么都没有，PX4 收不到任何 OFFBOARD 设定点。

**根因**：在 launch 里写了 `name='mavros'`。mavros 2.14.0 内部会为 container / router / uas / 每个插件各建一个 rclcpp 节点，而 launch 的 `name=` 会通过 `--ros-args -r __node:=mavros` 注入**整个进程**，导致所有节点被压成同名，直接崩溃。

**解决**：自定义 `mavros_uav1.launch.py`，**只写 `namespace='uav1'`，绝不写 `name=`**：

```python
Node(
    package='mavros',
    executable='mavros_node',
    namespace='uav1',          # 只有 namespace，没有 name=
    output='screen',
    parameters=[{
        'fcu_url': 'udp://:14550@10.41.10.2:14550',
        'tgt_system': 1,
        'tgt_component': 1,
        ...
    }],
)
```

### 坑 2：`/uav1/setpoint_raw/local` 订阅数是 0

**现象**：节点活着，位姿也正常，但发设定点飞机没反应。

**根因**：MAVROS 默认插件 allowlist 只有 9 个插件，**不包含 `setpoint_raw`**。OFFBOARD 模式全靠这个插件收设定点。改 `/opt/ros/.../px4_pluginlists.yaml` 要 sudo，不利于复现。

**解决**：把 allowlist 直接作为 launch 节点参数传入，等效于「官方默认 9 插件 + setpoint_raw」：

```python
'plugin_allowlist': [
    'sys_status', 'sys_time', 'command', 'imu',
    'local_position', 'manual_control', 'setpoint_position',
    'setpoint_raw',          # EGO 的 OFFBOARD 设定点全靠它
    'vision_pose*', 'odometry',
],
'plugin_denylist': ['*'],
```

实测 launch 传入的 `plugin_allowlist` 会覆盖 `/opt` 下的 yaml，且 `vision_pose* / odometry / local_position` 全部正常加载，零回归。

验收：

```bash
ros2 topic echo /uav1/state --once              # connected: true
ros2 topic info /uav1/setpoint_raw/local        # 应有订阅者
ros2 topic hz /uav1/setpoint_raw/local          # 桥接心跳应 ~30Hz
```

---

## 8. 第五步：EGO-Planner 接入真实雷达

EGO-Planner 原生是为仿真设计的（自带 map_generator / simulator）。板卡上把上游 `ego-planner-swarm` 的 `advanced_param.launch.py` 改写成了一个**自包含**的 `fastlio_mid360.launch.py`：

- 删掉所有仿真节点；
- 输入重映射到 FAST-LIO2 的真实话题；
- 保留 `drone_0_` 前缀，与下游桥接对齐；
- 目标点走 `/move_base_simple/goal`（RViz 2D Nav Goal 或命令行发布）。

启动方式（**按绝对路径 launch，因为文件不在包内**）：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY

ros2 launch ~/mid360_fastlio_ws/fastlio_mid360.launch.py \
  odom_topic:=/Odometry \
  cloud_topic:=/cloud_registered \
  max_vel:=0.3 max_acc:=0.2 \
  map_resolution:=0.20 inflation:=0.20 virtual_ceil:=3.0
```

输入重映射（核心几行）：

```python
remappings=[
    ('odom_world',      [odom_topic]),        # /Odometry
    ('grid_map/odom',   [odom_topic]),
    ('grid_map/cloud',  [cloud_topic]),       # /cloud_registered
    ('grid_map/pose',   '/unused_camera_pose'),# 没有深度相机，指向空话题
    ('grid_map/depth',  '/unused_depth'),
    ('goal_point',      '/move_base_simple/goal'),
    ('planning/bspline',['drone_', drone_id, '_planning/bspline']),
]
```

几个关键参数：

```python
{'fsm/flight_type': 1},                    # MANUAL_TARGET，收 /move_base_simple/goal
{'grid_map/resolution': map_resolution},
{'grid_map/obstacles_inflation': inflation},
{'grid_map/virtual_ceil_height': virtual_ceil},
{'grid_map/frame_id': 'world'},
{'manager/max_vel': max_vel},
{'manager/max_acc': max_acc},
```

同一份 launch 里还包含了 `traj_server` 节点，它把 B-spline 轨迹转成 `/drone_0_planning/pos_cmd`（类型 `quadrotor_msgs/PositionCommand`），所以**不需要单独再起 traj_server 会话**。

发一个目标点：

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

验收：

```bash
ros2 node list | grep ego_planner     # /drone_0_ego_planner_node 存活
ros2 topic hz /drone_0_planning/pos_cmd
```

---

## 9. 第六步：EGO → PX4 控制桥与 OFFBOARD 心跳

PX4 的 OFFBOARD 模式有个硬性要求：**必须持续收到设定点**（通常 >2Hz），否则几百毫秒后就自动退出 OFFBOARD 并切回原模式。所以不能只在发目标点时扔一条设定点，需要一个常驻的"心跳 + 指令转换"节点。

板卡上的 `ego_to_mavros/pos_cmd_bridge.py` 做三件事：

1. 订阅 `/drone_0_planning/pos_cmd`（`PositionCommand`），转换成 MAVROS 的 `PositionTarget`；
2. 以 30Hz 持续发布到 `/uav1/setpoint_raw/local`，即使没有新轨迹也维持心跳；
3. 可选地自动切 OFFBOARD / 解锁，并做完整的**限幅保护**。

启动参数（一键脚本里的保守值）：


```bash
ros2 launch ego_to_mavros pos_cmd_bridge.launch.py \
  mavros_namespace:=uav1 \
  pos_cmd_topic:=/drone_0_planning/pos_cmd \
  odom_topic:=/Odometry \
  setpoint_topic:=/uav1/setpoint_raw/local \
  publish_rate:=30.0 \
  max_vel_xy:=0.3 max_vel_z:=0.3 \
  max_acc_xy:=0.2 max_acc_z:=0.2 \
  min_z:=0.31 max_z:=2.8 \
  auto_offboard:=false auto_arm:=false auto_start_on_goal:=true \
  initial_takeoff_hold:=true initial_takeoff_height:=1.0
```

源码里的安全设计值得单独学习：

```python
def clamp_abs(value, limit):          # 对称限幅，防止 EGO 输出异常速度
    return max(-abs(limit), min(abs(limit), value))

def finite(*values):                  # NaN / Inf 检查，脏数据直接丢弃
    return all(math.isfinite(v) for v in values)
```

以及 `max_pos_cmd_age: 0.3` / `max_odom_age: 0.3` 的过期保护、`initial_takeoff_hold`（先原地悬停到 1m 再接管）、`hold_on_timeout`（超时保持而非乱飞）。**自主飞行代码里，这类"异常时该怎么办"的逻辑比规划算法本身更重要。**

---

## 10. 一键启动：6 个 screen 会话的编排

最终把所有节点收进 `start_all_mid360_ego.sh`，用 `screen` 管理。启动顺序是精心排过的：

| 顺序 | 会话名                 | 节点                        | 为什么排在这                               |
| -- | ------------------- | ------------------------- | ------------------------------------ |
| 1  | `mid360_mavros`     | MAVROS（自定义 launch）        | **必须第一个**，插件加载要 1~2 分钟               |
| 2  | `mid360_lidar`      | livox_ros_driver2         | 数据源，先起来                              |
| 3  | `mid360_fastlio`    | FAST-LIO2                 | 依赖雷达数据                               |
| 4  | `mid360_bridge`     | external_odom_to_mavros   | 依赖 `/Odometry`                       |
| 5  | `mid360_ego`        | EGO-Planner + traj_server | 依赖 `/Odometry` + `/cloud_registered` |
| 6  | `mid360_ego_bridge` | pos_cmd_bridge            | 依赖 EGO 输出和 MAVROS                    |

脚本开头有几行"保命"设置，每一行背后都是一次事故：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 全链路 DDS 必须一致
unset CYCLONEDDS_URI ROS_LOCALHOST_ONLY        # 旧终端残留会指向已删文件 → 全部 exit -6
```

启动前统一 source 所有工作区，避免"这个包在那个终端能找到、这个终端找不到"：

```bash
SOURCE_ALL="source /opt/ros/humble/setup.bash; \
            source ~/ros2_ws/install/setup.bash; \
            source ~/mid360_fastlio_ws/install/setup.bash; \
            source ~/mid360_ws/install/setup.bash"
```

每个会话用 `screen -dmS <name> bash -c "...; exec bash"` 起，`exec bash` 保证节点退出后会话还在，方便进去看报错。停止时统一 `screen -X -S <name> quit` 再 `pkill` 兜底。

---

## 11. 试飞流程与验收清单

按这个顺序检查，不要跳步：

```bash
# 1. 定位链路
ros2 topic hz /Odometry                      # ~10Hz
ros2 topic hz /uav1/vision_pose/pose         # >20Hz

# 2. 控制链路
ros2 topic hz /uav1/setpoint_raw/local       # ~30Hz，空窗 <0.3s（关键！）
ros2 topic echo /uav1/state --once           # connected: true

# 3. 规划链路
ros2 topic list | grep planning              # /drone_0_planning/pos_cmd
ros2 node list | grep ego_planner

# 4. 发目标点（第一次 <1m，先不要放障碍物）
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"

# 5. 切 OFFBOARD + 解锁（设定点要先于 offboard 发出）
alias offboard='ros2 service call /uav1/set_mode mavros_msgs/srv/SetMode "{custom_mode: '\''OFFBOARD'\''}"'
alias arm='ros2 service call /uav1/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"'
```

首次真机参数一定要保守：EGO 最大速度 0.3m/s、加速度 0.2m/s²，桥接限速限高同量级，目标点小于 1 米。确认能稳定悬停、轨迹平滑之后，再逐步放大到 0.5 / 0.8。

### 真机实测：悬停与避障

下面是板卡实机跑出来的悬停与避障录像（已去除音轨），可以直观看到目标点发布后起飞、稳定悬停与自主避障的全过程：

<video src="/videos/rk3588-hover-and-avoidance.mp4" controls preload="metadata" width="100%" style="max-width: 960px; border-radius: 6px;">  
  你的浏览器不支持 HTML5 视频，<a href="/videos/rk3588-hover-and-avoidance.mp4">点此下载观看</a>。  
</video>

---

## 12. 踩坑速查表

| 现象                               | 根因                                                       | 解决                                           |
| -------------------------------- | -------------------------------------------------------- | -------------------------------------------- |
| MAVROS 启动即 exit -6               | launch 里写了 `name='mavros'`，注入 `--ros-args -r __node` 压同名 | 自定义 launch 只用 `namespace`，删掉 `name=`         |
| `/uav1/setpoint_raw/local` 订阅数 0 | 不在 MAVROS 默认插件 allowlist                                 | launch 传 `plugin_allowlist` 含 `setpoint_raw` |
| 所有节点集体 exit -6                   | `CYCLONEDDS_URI` / `ROS_LOCALHOST_ONLY` 残留，指向已删文件        | 启动脚本开头 `unset`                               |
| 节点互相发现不了                         | 各终端 RMW 不一致                                              | 统一 `rmw_cyclonedds_cpp`                      |
| 雷达话题无数据、不报错                      | 网卡 IP 与配置 `host_ip` 不一致，驱动 bind 失败                       | `switch_lidar_ip.sh` 对齐并保留其它网段               |
| 建图一直飘                            | IMU 未预热 / 外参时间偏移没标定                                      | 上电静置 10s；用 LI-Init 标定后填回 YAML                |
| EGO launch 提示找不到包                | 主 launch 不在 `ego_planner` 包内                             | 用绝对路径 `ros2 launch <abs path>`               |
| 没有深度相机 EGO 报错                    | grid_map 订阅 pose/depth                                   | remap 到 `/unused_*` 空话题                      |
| 切 OFFBOARD 后立刻掉模式                | 设定点没持续发                                                  | pos_cmd_bridge 以 30Hz 发心跳                    |
| 注释里的雷达 IP 与配置不符                  | 脚本注释是旧值                                                  | 以驱动加载的 JSON 为准                               |

---

## 13. 附录 A：点云通过 UDP 转给地面站

`ros2_to_udp.py` 订阅 `/cloud_registered`，降采样后打包发给 `192.168.144.11:8888` 做可视化：

- 每帧最多 2000 点，5Hz 限流（防止图传卡死）；
- int16 量化，scale=0.025（精度 2.5cm，量程 ±819m）；
- 32 字节包头（magic `0xABCD` + 帧号 + 原点 + scale + 时间戳 + 分片标志 + CRC16）；
- 每包最多 195 点，首片/末片标志位便于接收端重组。

配直连路由，确保流量走对应网口、不经过网关：

```bash
sudo ip addr flush dev eth0
sudo ip addr add 192.168.144.20/24 dev eth0
sudo ip link set eth0 up
sudo ip route replace 192.168.144.0/24 dev eth0 src 192.168.144.20
ip route get 192.168.144.11        # 必须显示 dev eth0 src ...，不能有 via
```

> 排错经验：`ip route get` 的输出里一旦出现 `via`，说明包走了默认网关而不是直连，图传会时通时断。

---

## 14. 附录 B：FAST-LIVO2 视觉增强方案（可选）

如果还要加视觉，`fastlivo2_isolated` 里是一套完整隔离环境：

```bash
cd ~/fastlivo2_isolated
./start_all_fastlivo2_screen.sh          # 启动全套（含网卡 IP 对齐）
./start_all_fastlivo2_screen.sh net      # 只看网络状态，不启动
./start_all_fastlivo2_screen.sh stop     # 停会话 + 还原网卡 IP
```

它在 FAST-LIO2 基础上加了 RealSense 相机（`img_time_offset: 0.1`，`Rcl/Pcl` 是相机到雷达的外参），输出话题变成 `/aft_mapped_to_init`。注意桥接节点的 `ODOM_TOPIC` 要跟着改：

```bash
ODOM_TOPIC=/aft_mapped_to_init ./start_all_fastlivo2_screen.sh
```

日志实时写入 `~/fastlivo_realtime.log`，排查用 `tail -f` 即可。

---

## 15. 附录 C：用 Foxglove 远程看点云与规划轨迹

前面附录 A 的点云 UDP 转发是"把点云发给地面站自定义程序"的路子。如果只是想在电脑上**直接看点云、看轨迹、顺便调参数**，更省事的办法是跑一个 Foxglove bridge，然后用浏览器连上去——不用装 ROS，也不用自己解析点云包。

这套方案在 Windows 笔记本上实测可用，浏览器打开一个网页就能实时看到 `/cloud_registered` 全局地图和 EGO 的 B-spline 规划轨迹。

### 15.1 为什么选 Foxglove bridge 而不是 rosbridge

- `rosbridge_suite` 走 JSON，点云这种大数据量话题序列化开销大，卡；
- `foxglove_bridge` 走 **WebSocket + 二进制（ROS2 原生 CDR）**，点云传输效率高得多；
- 板卡上 `/opt/ros/humble/share` 里已自带 `foxglove_bridge`（实测版本 3.5.0），不用额外编译。

### 15.2 板端启动 bridge

用一个**独立的 screen 会话**，不要和主链路的 6 个会话混在一起——bridge 是调试工具，随时可能重启，不该影响飞行链路：

```bash
screen -dmS foxglove bash -c "
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/mid360_ws/install/setup.bash
source ~/mid360_fastlio_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
exec bash
"

screen -ls | grep foxglove          # 确认会话在
ss -ltnp | grep 8765                # 确认端口在监听
```

两个细节：

1. **RMW 仍要设成 `rmw_cyclonedds_cpp`**，和主链路保持一致，否则 bridge 发现不了那些节点发布的话题；
2. 启动后日志会打印 `Server listening on port 8765`，并把当前所有话题逐个 `Advertising new channel`。实测一次启动 advertise 了 126 个 channel，`/cloud_registered`、`/drone_0_planning/bspline`、`/Odometry`、`/grid_map/occupancy` 都在其中。

### 15.3 电脑端连接

浏览器打开 https://app.foxglove.dev ，新建连接，地址填：

```
ws://192.168.110.156:8765
```

连上后左侧是话题列表，中间加一个 **3D 面板**，把需要看的话题拖进去：

| 想看什么 | 话题 | 面板里怎么配 |
| --- | --- | --- |
| 全局点云地图 | `/cloud_registered` | Point size 调小（实测 1.5 比较清楚），Color mode 选 **Intensity**，Palette 用 Turbo 或 Rainbow，能看出发散扫描的层次 |
| 机体坐标系点云 | `/cloud_registered_body` | 同上，用于看最近一帧的扫描 |
| 占据栅格地图 | `/grid_map/occupancy`、`/drone_0_grid/grid_map/occupancy_inflate` | 当点云显示，能看出 EGO 的膨胀边界 |
| 规划轨迹 | `/drone_0_planning/bspline` | 作为曲线/标记显示，配合目标点看轨迹生成 |
| 定位 | `/Odometry`、`/uav1/local_position/pose` | 看轨迹是否平滑 |

实测效果：点云地图完整显示出来，一条 B-spline 规划轨迹从机体当前位置弯向目标点，右侧终端用 `ros2 topic pub --once /move_base_simple/goal` 发目标点即可触发重新规划。

![Foxglove 远程查看点云与规划轨迹](/images/foxglove-remote-viz.jpg)

### 15.4 发目标点触发规划

在板卡终端发目标点（和第 8 节一样），Foxglove 里能立刻看到新轨迹：

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0, z: 1.0}, orientation: {w: 1.0}}}"
```

这是调试 EGO 最舒服的方式：左边改参数、右边发目标、浏览器里实时看轨迹变化，不用反复开 RViz 截图。

### 15.5 两个实测踩坑

1. **点云历史深度被截断**：日志里出现
   `Limiting history depth for topic '/cloud_registered' to 10 (was 20). You may want to increase the max_qos_depth parameter value.`
   这是 bridge 默认 `max_qos_depth` 太小。点云本身是"当前状态"型数据，深度 10 其实够用；如果确实要缓存更多帧，给 launch 加参数 `max_qos_depth:=...`。

2. **连不上先查两件事**：`ss -ltnp | grep 8765` 看端口是否在监听；`screen -r foxglove` 看 bridge 进程有没有崩。另外确认电脑和板卡在同一网段能互通——WebSocket 走的是 TCP 8765，和 MAVROS 的 UDP 14550、点云转发的 UDP 8888 是三件互不相关的事，别混。

### 15.6 和附录 A 的选择建议

| 需求 | 用哪个 |
| --- | --- |
| 快速看点云/轨迹、调参数、跨设备 | **Foxglove bridge**（本章）——零客户端开发，浏览器直连 |
| 要把点云接进自己的地面站程序、自定义协议 | **UDP 转发**（附录 A）——可控字段和频率，适合定制 |

两者可以同时跑，互不冲突。
