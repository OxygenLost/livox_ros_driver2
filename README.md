# Livox ROS Driver 2 - Auto-Level Edition

基于 Livox ROS Driver 2 官方驱动的增强版本，新增 **MID360 / MID360S / MID360L 自动水平校准**功能，支持雷达任意角度安装（倾斜/倒置），自动矫正输出的点云与 IMU 数据为水平坐标系，无需手动测量安装角度。

## 核心特性

### ✨ 自动水平校准（Auto-Level Calibration）

- **问题**：MID360 倾斜/倒置安装时，安装角度难以精确测量
- **解决方案**：
  - 启动时静止采集 IMU 加速度，自动估计 roll/pitch（忽略 yaw）
  - 同时旋转点云与 IMU 输出，保证 faster-lio 等 SLAM 算法正常耦合
  - 可选：自动更新 faster-lio yaml 中的 `extrinsic_T`（IMU-LiDAR 平移外参）
  - 校准结果保存到 `config/MID360_config_calib.json`，后续直接使用

### 📦 兼容原版功能

- 支持 HAP / MID360 / MID360S / MID360L / 混合多雷达
- PointCloud2 / CustomMsg / PCL 格式
- ROS1 (Noetic) / ROS2 (Foxy/Humble)

---

## 快速开始

### 1. 安装依赖

#### ROS (推荐 Noetic)
```bash
# Ubuntu 20.04
sudo apt install ros-noetic-desktop-full
```

#### Livox SDK 2
参考官方文档：[Livox-SDK2 安装说明](https://github.com/Livox-SDK/Livox-SDK2)。

- MID360S 需要 SDK2 1.3.0 及以上。
- **MID360L 必须使用 SDK2 1.5.0 及以上**（设备 `172.16.120.95` 使用 SDK2 1.5.2 验证）。旧 SDK2 1.3.1 不支持设备类型 `kLivoxLidarTypeMid360l = 41`，仅替换 JSON 无法适配。
- SDK2 升级后需要重新编译本仓库的 ROS1/ROS2 驱动。ROS1 链接 SDK 静态库，不能只覆盖旧节点的配置文件。

### 2. 编译

```bash
# ROS1
cd /path/to/your_workspace
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1
```

```bash
# ROS2 (Foxy/Humble)
source /opt/ros/foxy/setup.bash
colcon build
```

### 3. 使用

#### 方式 A：一键校准 + 可视化（推荐首次使用）

```bash
# 确保雷达已连接并静止，运行验证脚本
source devel/setup.bash
./src/livox_ros_driver2/scripts/calib_and_visualize.sh
```

该脚本会：
1. 自动校准（3秒静止采样）
2. 生成 `config/MID360_config_calib.json`
3. 启动驱动 + RViz 可视化

#### 方式 B：仅校准（不启动可视化）

```bash
# 需要先启动 roscore 和 livox 驱动（使用原始 config）
roslaunch livox_ros_driver2 msg_MID360.launch

# 新终端运行校准
python3 src/livox_ros_driver2/scripts/mid360_autolevel_calib.py \
  --duration 3.0 \
  --imu-topic /livox/imu \
  --lidar-topic /livox/lidar \
  --fasterlio-yaml /path/to/faster-lio/config/mid360.yaml
```

校准完成后，使用校准配置启动驱动：

```bash
roslaunch livox_ros_driver2 autolevel_MID360.launch
```

#### 方式 C：原版使用方式（不校准）

```bash
# MID360 + PointCloud2 + RViz
roslaunch livox_ros_driver2 rviz_MID360.launch

# MID360 + CustomMsg（faster-lio 建图）
roslaunch livox_ros_driver2 msg_MID360.launch xfer_format:=1

# MID360S + CustomMsg
roslaunch livox_ros_driver2 msg_MID360s.launch xfer_format:=1

# MID360L + CustomMsg
roslaunch livox_ros_driver2 msg_MID360l.launch xfer_format:=1

# MID360L + PointCloud2 + RViz
roslaunch livox_ros_driver2 rviz_MID360l.launch

# MID360L + ROS2
ros2 launch livox_ros_driver2 msg_MID360l_launch.py
```

---

## 校准参数说明

`mid360_autolevel_calib.py` 命令行参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--base-config` | 基础配置 JSON 路径 | `config/MID360_config.json` |
| `--output-config` | 校准结果输出文件，避免不同型号互相覆盖 | `config/MID360_config_calib.json` |
| `--duration` | 校准采样时长（秒） | `3.0` |
| `--timeout` | 数据检查超时（秒） | `8.0` |
| `--imu-topic` | IMU topic 名称 | `/livox/imu` |
| `--lidar-topic` | 点云 topic 名称 | `/livox/lidar` |
| `--target-ip` | 指定雷达 IP（多雷达场景） | 空（全部） |
| `--fasterlio-yaml` | faster-lio yaml 路径（可选） | 空（不更新） |

---

## 配置文件说明

### 核心配置参数

#### `config/MID360_config.json`（或自定义路径）

本仓库的通用 MID360 配置同时注册 `MID360`、`Mid360s`、`Mid360l` 三个 SDK 设备类型，可接收普通 MID360（`dev_type=9`）、MID360S（`dev_type=35`）或 MID360L（`dev_type=41`），设备发现和数据协议由 SDK2 处理。提供独立的 `config/MID360s_config.json`、`config/MID360l_config.json` 及对应 ROS1/ROS2 launch 供显式启动。

```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.1.102",
      "point_data_ip": "192.168.1.102",
      "imu_data_ip": "192.168.1.102"
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.191",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

**关键参数**：
- `host_net_info.xxx_data_ip`：主机 IP（需与雷达同网段）
- `Mid360s` / `Mid360l` 的 `host_net_info` 为**数组**，其中的 `host_ip` 是主机地址；不能套用老 MID360 的 `cmd_data_ip` 配置结构
- `lidar_configs[].ip`：雷达 IP
- `extrinsic_parameter`：外参（校准后会自动填充 roll/pitch）

#### MID360L 设备专属配置与自动水平校准

`config/MID360l_config.json` 提供 `172.16.120.95` 首次验证的网络参数：主机 `192.168.1.5`、雷达 `192.168.1.110`。其 roll/pitch 为**零外参示例**，不是该设备已标定的安装角度。其他设备使用前必须调整两个 IP，不能照搬安装外参。

针对单台 MID360L 静止校准时，明确指定输出文件，防止覆盖旧机型共用的 `MID360_config_calib.json`：

```bash
python3 src/livox_ros_driver2/scripts/mid360_autolevel_calib.py \
  --base-config src/livox_ros_driver2/config/MID360l_config.json \
  --output-config src/livox_ros_driver2/config/MID360l_config_calib.json \
  --target-ip 192.168.1.110

# 继续使用原有 ROS1 导航启动入口，只替换配置文件路径
roslaunch livox_ros_driver2 autolevel_MID360.launch \
  user_config_path:=/absolute/path/to/MID360l_config_calib.json
```

设备 `172.16.120.95` 当前热更新验证保留的原安装外参为 roll=0.649944294605901°、pitch=31.53081018368778°、yaw=0°，存放于设备自己的容器配置中；**本仓库不以单台设备的安装外参覆盖其他 MID360/MID360S 默认配置**。在新固件镜像上需要一并带入对应设备的校准配置。

### Launch 文件参数

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `xfer_format` | 点云格式：`0`=PointCloud2，`1`=CustomMsg | faster-lio 用 `1` |
| `output_type` | 输出方式：`0`=topic，`1`=bag only | 建图用 `0` 或 `1` |
| `multi_topic` | 多雷达独立 topic：`0`=共享，`1`=独立 | 单雷达用 `0` |
| `publish_freq` | 发布频率（Hz） | `10.0` |

---

## 工作原理

### 校准流程

1. **启动临时驱动**：使用"外参清零"的临时配置（raw IMU/点云）
2. **数据检查**：等待点云/IMU topic 有数据（超时则报错）
3. **静止采样**：订阅 IMU，采集 N 个加速度样本（默认 3 秒）
4. **姿态估计**：基于重力方向计算 roll/pitch（公式：`roll=atan2(ay,az), pitch=atan2(-ax,sqrt(ay²+az²))`）
5. **生成配置**：写入 `config/MID360_config_calib.json`（仅改 roll/pitch，保留原始 yaw/xyz）
6. **更新 faster-lio**（可选）：对 `extrinsic_T` 做同步旋转变换
7. **退出**：校准完成，停止临时驱动

### 关键设计

- **点云与 IMU 同步旋转**：驱动已在 `pub_handler.cpp` 中实现"外参旋转同时作用到点云与 IMU 输出"（L161-172），保证 SLAM 算法正常耦合
- **最小 C++ 改动**：保留自动水平校准和已有时序/队列诊断逻辑，补充 MID360S/MID360L 的设备类型及线号处理；型号发现和网络通道由 Livox-SDK2 完成
- **单位兼容**：自动识别 IMU 加速度单位（`1g` 或 `9.81m/s²`）

---

## 常见问题

### Q1：校准时报错 "timeout waiting topic: /livox/imu"
- **原因**：雷达未连接或网络配置错误
- **排查**：
  ```bash
  rostopic list  # 确认 /livox/imu 存在
  rostopic hz /livox/imu  # 确认有数据
  ```

### Q2：校准时报错 "robot not static? gyro_norm: xxx"
- **原因**：机器人在移动
- **解决**：保持机器人静止 5 秒后重新校准

### Q3：校准时报错 "acc norm abnormal: xxx"
- **原因**：IMU 数据异常或单位不匹配
- **排查**：检查 `rostopic echo /livox/imu -n 1`，加速度模长应约为 1 或 9.81

### Q4：faster-lio 建图效果仍然差
- **原因**：可能是 `extrinsic_T` 未更新或雷达盲区设置不当
- **解决**：
  1. 校准时加 `--fasterlio-yaml` 参数自动更新外参
  2. 检查 `faster-lio/config/mid360.yaml` 中 `blind` 参数（推荐 1.5-2.0）

---

## 与官方版本差异

本仓库以 [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) v1.2.4 为基础，**选择性回移**官方 `13eb05e`（MID360S）和 `2144554`（MID360L）中的相关能力；并非完整合并官方 1.2.8 全部变更，同时保留本仓库的自动水平校准、点云与 IMU 旋转和诊断定制：

- `scripts/mid360_autolevel_calib.py`：自动水平校准脚本
- `scripts/calib_and_visualize.sh`：一键校准 + 可视化验证
- `launch_ROS1/autolevel_MID360.launch`：使用校准配置的启动文件
- `config/MID360_config_calib.json`：校准输出配置（自动生成）
- `config/MID360s_config.json`：MID360S 独立配置
- `config/MID360l_config.json`：MID360L 独立配置（示例为 95 设备的雷达 IP，零外参）
- `launch_ROS1/msg_MID360s.launch` / `rviz_MID360s.launch`：ROS1 MID360S 启动入口
- `launch_ROS1/msg_MID360l.launch` / `rviz_MID360l.launch`：ROS1 MID360L 启动入口
- `launch_ROS2/msg_MID360s_launch.py` / `rviz_MID360s_launch.py`：ROS2 MID360S 启动入口
- `launch_ROS2/msg_MID360l_launch.py` / `rviz_MID360l_launch.py`：ROS2 MID360L 启动入口
- `MID360_config*.json` 同时注册 `MID360` / `Mid360s` / `Mid360l`，现有自动校准启动链可按设备发现处理

没有直接 merge 官方 master，避免覆盖本仓库已有的自动校准、IMU/点云同步旋转和 Faster-LIO 外参同步逻辑。

---

## License

MIT License (与官方版本相同)

## 致谢

- Livox 官方驱动：[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- faster-lio：[faster-lio](https://github.com/gaoxiang12/faster-lio)
