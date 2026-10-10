# G1 EDU 本体 IMU 采集 + FAST-LIO2 bag 数据集

目标：在 **Unitree G1 EDU** 的机载电脑上（ROS2），用 **G1 本体 IMU** 作为
惯性源，配合 LiDAR 点云，录制一份可直接喂给 **FAST-LIO2** 的 bag 数据集。

> 前提说明：FAST-LIO2 是 LiDAR-Inertial 里程计，**必须有 LiDAR 点云**。
> "路线 A"（用本体 IMU 而不是雷达自带 IMU）只是把惯性源从雷达换成 G1，
> 点云仍然需要。若你根本没有 LiDAR，FAST-LIO2 跑不起来，需改用 VIO 类算法。

---

## 1. 原理：为什么需要一个桥接

G1（`unitree_hg` 消息包）**没有独立的 IMU 话题**：

| 项目 | 内容 |
|---|---|
| DDS 话题 | `rt/lowstate`（高频）/ `rt/lf/lowstate`（低频） |
| 消息类型 | `unitree_hg/msg/LowState` |
| IMU 字段 | `imu_state`：`quaternion[4](w,x,y,z)`、`gyroscope[3]`、`accelerometer[3]`、`rpy[3]` |
| 时间戳 | **无硬件时间戳**，LowState 里只有 `tick` |

FAST-LIO2 只吃 `sensor_msgs/Imu`，所以数据链是：

```
rt/lowstate (DDS)  ──►  g1_imu_bridge.py  ──►  /g1/imu (sensor_msgs/Imu)
                                                        │
/livox/lidar (PointCloud2)  ──────────────────────────► FAST-LIO2  ──► /Odometry, map
                                                        └──────────► ros2 bag record
```

---

## 2. 文件说明

| 文件 | 用途 |
|---|---|
| `g1_imu_bridge.py` | 订阅 `rt/lowstate` → 发布 `/g1/imu` (`sensor_msgs/Imu`) |
| `record_g1_bag.sh` | 录制点云 + G1 IMU（+ 可选机器人状态/Fast-LIO 输出）的脚本 |
| `fastlio_mid360_g1imu.yaml` | FAST-LIO2 配置模板（MID-360 + G1 IMU） |

---

## 3. 前置依赖

在 G1 机载电脑上：

1. **ROS2**（Humble/Jazzy 均可，与机器人现有环境保持一致）。
2. **unitree_sdk2py**（提供对 `rt/lowstate` 的直接 DDS 订阅）：
   ```bash
   # 需要先编译安装 cyclonedds 0.10.x，再安装 python 包
   git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
   cd cyclonedds && mkdir build install && cd build
   cmake .. -DCMAKE_INSTALL_PREFIX=../install && cmake --build . --target install

   cd ~ && git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
   cd unitree_sdk2_python
   export CYCLONEDDS_HOME=~/cyclonedds/install
   pip3 install -e .
   ```
3. **LiDAR 驱动**：例如 `livox_ros_driver2` + 配置好的 `MID360_config.json`，
   能稳定发布 `/livox/lidar`。
4. **FAST-LIO2 (ROS2)**：如 `Ericsii/FAST_LIO_ROS2`（本配置模板即按它的字段结构写）。

网络：DDS 必须与机器人同网段、同 `ROS_DOMAIN_ID`，并用 `CYCLONEDDS_URI`
指定机器人内网网卡（G1 内网典型为 `192.168.123.x`）。

```bash
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp2s0" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```
> 把 `enp2s0` 换成机载电脑连接机器人内网的网卡名（用 `ip -br addr` 查）。

---

## 4. 操作步骤

### 步骤 0：确认能读到 LowState
先跑官方示例验证 DDS 通路（读得到再谈桥接）：
```bash
python3 ~/unitree_sdk2_python/example/g1/low_level/g1_low_level_example.py enp2s0
```

### 步骤 1：启动 IMU 桥接
```bash
# 终端 1
python3 g1_imu_bridge.py --ros-args \
  -p interface:=enp2s0 \
  -p dds_topic:=rt/lowstate \
  -p ros_topic:=/g1/imu \
  -p frame_id:=imu_link
```
桥接会打印 `IMU rate ~ xxx Hz`，用于确认频率（FAST-LIO2 建议 ≥100Hz，200Hz 左右最佳）。

### 步骤 2：单话题验证
```bash
# 终端 2
ros2 topic hz /g1/imu
ros2 topic echo --once /g1/imu
```
检查角速度/加速度在静止时应接近 `0` 和 `~9.8`。

### 步骤 3：启动 LiDAR 驱动并验证点云
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 topic hz /livox/lidar        # 约 10Hz
```

### 步骤 4：录包
```bash
# 只录原始输入（推荐先离线跑 FAST-LIO2）
LIDAR_TOPIC=/livox/lidar G1_IMU_TOPIC=/g1/imu ./record_g1_bag.sh

# 若实时跑 FAST-LIO2，同时录它的输出便于对照
RECORD_FASTLIO_OUTPUT=1 ./record_g1_bag.sh
```
采集动作建议：起录后**原地静止 5~10 秒**（供初始化），低速平移，
避免原地快速打转，尽量回到起点附近再静止数秒，最后 Ctrl-C。

### 步骤 5：验收
```bash
ros2 bag info <bag目录>
```
确认：`/livox/lidar` 约 10Hz、`/g1/imu` 频率与桥接一致、时长与实测相符、
目录下没有 `.active`。

### 步骤 6：离线跑 FAST-LIO2
```bash
# 终端 1
ros2 launch fast_lio mapping_mid360.launch.py \
  config_file:=$(pwd)/fastlio_mid360_g1imu.yaml   # 端口不同时按实际参数名传
# 终端 2
ros2 bag play --clock <bag目录>
```
看 `IMU Initial Done` 与 `/Odometry` 是否稳定、地图有无重影/双墙。

---

## 5. 必须处理的两个"坑"

### (1) 时间同步（最大的风险）
- G1 本体 IMU **没有硬件时间戳**，`g1_imu_bridge.py` 用**主机接收时间**打标。
- 要让时间基准一致，LiDAR 驱动也应使用**同一台机载电脑的主机时间**打标。
- 若轨迹明显滞后/超前，用 `time_offset_lidar_to_imu` 标定（可用 `LI-Init` 或手动扫参）。

### (2) 外参标定（`extrinsic_T` / `extrinsic_R`）
- FAST-LIO2 的状态原点在 **IMU 系**，配置里的外参是 **LiDAR → IMU** 的变换。
- G1 IMU 在躯干/pelvis，LiDAR 通常装在头部或躯干前方，**两者不重合**，
  必须实测或标定，不能用单位阵糊弄，否则点云会"甩"、地图扭曲。
- 标定选项：卷尺量平移 + 水平仪/AprilTag 估角度，或用 `LI-Init` 做自动初始化。

---

## 6. 参考

- `unitreerobotics/unitree_ros2` — 官方消息定义、`read_low_state_hg.cpp`、`record_bag.cpp`
- `unitreerobotics/unitree_sdk2_python` — Python DDS 订阅
- `Ericsii/FAST_LIO_ROS2` — FAST-LIO2 的 ROS2 端口与 `config/mid360.yaml`
- `linchengli86/unitree-g1-onboard-SLAM-v0.1-mapping` — G1 机载 SLAM 实作参考







再加一个关节编码器 +  视觉一起采集。
