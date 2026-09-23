# merge_cloud

把两路激光雷达（Odin1 + Livox）的点云标定对齐、运动补偿、裁剪切块后合成一路输出。

输入是 Odin1 的主点云和 Livox 的原始点云，输出是单帧的 `/merged_cloud`。

## 目录结构

```
merge_cloud/
├── CMakeLists.txt
├── package.xml
├── config/merge_cloud.yaml       # 话题名、外参、盲区球参数
├── launch/merge_cloud.launch.py
└── src/merge_cloud_code.cpp      # 单文件实现，全部逻辑在 MergeCloudNode 类里
```

## 依赖与安装

环境：**Ubuntu 22.04 + ROS 2 Humble**，构建类型 `ament_cmake`。

### 1. ROS 2 Humble

若尚未安装，按官方文档装 `ros-humble-desktop` 或 `ros-humble-ros-base`：
<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

### 2. 本包依赖

全部都在 apt 里，一条命令装完：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-ros \
  ros-humble-rclcpp \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-launch \
  ros-humble-launch-ros \
  libeigen3-dev
```

`libpcl-dev` 会作为 `ros-humble-pcl-ros` 的依赖自动装上（Humble 对应 PCL 1.12）。

如果你的包放在一个 colcon workspace 里，也可以让 rosdep 照 `package.xml` 解析全部依赖：

```bash
sudo rosdep init && rosdep update    # 只需首次执行
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. livox_ros_driver2（可选）

apt 里没有这个包，需要源码编译。这里有一个容易漏掉的前置依赖：**必须先编译安装 Livox-SDK2**。
驱动包的 `CMakeLists.txt` 是直接从 `/usr/local/lib` 找 `liblivox_lidar_sdk_static.a` 的
（`find_library(... /usr/local/lib)`），并不会通过 git submodule 自动拉取 SDK。

```bash
# ① 先装 Livox-SDK2，它会安装到 /usr/local
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install

# ② 再编译驱动包，注意必须放在 workspace 的 src/ 下
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble      # 该仓库自带的构建脚本，参数填 ROS 发行版名
```

编译完成后从它所在的 workspace source 出环境即可。

> **实际上可以不装。** `merge_cloud_code.cpp` 没有 include 任何 livox 头文件——源码里唯一出现的
> "livox" 是默认话题名 `/livox/lidar/pointcloud`。`livox_ros_driver2` 只出现在
> `CMakeLists.txt` 的 `find_package` / `ament_target_dependencies` 和 `package.xml` 的 `<depend>` 中，
> 编译本节点并不需要它。若不想引入这个依赖，把这 3 处删掉即可：驱动是独立节点，与本包只通过
> 话题交互。

### 4. 验证依赖就位

```bash
ros2 pkg prefix pcl_ros
ros2 pkg prefix livox_ros_driver2    # 若按上面的说明跳过则忽略这条
```

## 构建与运行

在包含 `merge_cloud` 的 colcon workspace 根目录下：

```bash
colcon build --packages-select merge_cloud
source install/setup.bash
ros2 launch merge_cloud merge_cloud.launch.py
```

## 话题

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 订阅 | `cloud1_topic`（默认 `/odin1_/cloud_slam`） | `sensor_msgs/PointCloud2` | SensorDataQoS |
| 订阅 | `cloud2_topic`（默认 `/livox/lidar/pointcloud`） | `sensor_msgs/PointCloud2` | SensorDataQoS |
| 订阅 | `odom_topic`（默认 `/odin1_/odometry_highfreq`） | `nav_msgs/Odometry` | SensorDataQoS |
| 发布 | `/merged_cloud` | `sensor_msgs/PointCloud2` | depth 10 |

## 参数

`config/merge_cloud.yaml` 里的值会覆盖代码里的默认值。

### 话题与坐标系

| 参数 | yaml 当前值 | 代码默认值 | 说明 |
|---|---|---|---|
| `cloud1_topic` | `/odin1_/cloud_slam` | `/odin1_/cloud_raw` | 主点云，**基准**，不做去畸变 |
| `cloud2_topic` | `/livox/lidar/pointcloud` | 同 | 副点云，**去畸变**，它的到达触发一次合并 |
| `odom_topic` | `/odin1_/odometry_highfreq` | 同 | Odin1 的里程计，用于去畸变；缺失则跳过补偿 |
| `frame_id` | `front_odin1` | `front_odin1` | 输出点云 `header.frame_id`，仅作标签，代码不做 TF 查询 |

### 外参

两组都是 `(roll, pitch, yaw, tx, ty, tz)`，旋转单位为弧度，按 `Rz(yaw)·Ry(pitch)·Rx(roll)` 组合，平移单位米。

| 参数 | yaml 当前值 | 含义 |
|---|---|---|
| `roll1` `pitch1` `yaw1` | `0 0 0` | cloud1 → 输出系 的旋转（当前为单位阵） |
| `tx1` `ty1` `tz1` | `0.02 0.03447 0.02174` | cloud1 → 输出系 的平移 |
| `roll2` `pitch2` `yaw2` | `0.5236 0 -1.5708` | cloud2 → cloud1 系 的旋转（即 +30° 横滚、-90° 偏航） |
| `tx2` `ty2` `tz2` | `-0.31436 0.0165 0.0373` | **在 cloud1 系下表达**的光心间距 |

注意 `cloud1_static` 当前旋转为单位阵，因此它只决定输出整体原点、**不影响两雷达的相对几何**——
看到重影时只能调 `roll2…tz2`，调 `roll1/pitch1/yaw1` 无效。

### 裁剪与盲区

| 参数 | yaml 当前值 | 说明 |
|---|---|---|
| `crop_half_size` | **未配置** | XY 方向半边长（米），**实际生效值为代码默认的 5.0**。不限 Z |
| `blind_sphere_enable` | `true` | 是否做盲区球过滤 |
| `blind_sphere_radius` | `0.3` | 球半径（米） |
| `blind_sphere_cx` `cy` `cz` | `0 0 0` | 球心相对雷达原点的偏移（米） |
| `blind_sphere_follow_odom` | `true` | **已失效**，保留仅为兼容 yaml；点云本身就在随机器人移动的系里，球心恒随原点。配成 `false` 会在启动时打一条 WARN |

## 处理流程

每一帧 Livox 到达时执行一次：

```
① 时间配对   cloud1 从 50 帧环形缓冲里挑时间戳离 cloud2 最近的一帧
② 字段归一   两路都转成 pcl::PointXYZI（cloud1 优先 rgb→灰度，cloud2 用 intensity）
③ 变换链     见下表
④ 拼接       *merged = *cloud1 + *cloud2        ← 纯 append，无配准/去重
⑤ 后处理     XY 裁剪 ±crop_half_size（中心为原点）+ 盲区球过滤
⑥ 发布       header.frame_id = frame_id, header.stamp = cloud2 的时间戳
```

### 变换链

```
cloud1:  p₁  --T₁-->  p_out

cloud2:  p₂  --T₂-->  Odin1 系（各点各自采集时刻）
             --去畸变-->  Odin1 系（统一对齐到扫描起始时刻）
             --T₁-->  p_out
```

去畸变是把每个点乘上 `T_world(t_ref)⁻¹ · T_world(t_pt)`，其中 `t_ref` 取 cloud2 帧头时间戳。
这是**纯帧内运动补偿**，结果仍留在 Odin1 系，不会被推到里程计的世界系去。静止时该变换恒等。

逐点时刻优先读点云里的 `timestamp` / `time` / `t` 字段（按 **uint32 纳秒偏移**解析）；读不到就退化为
「假设扫描时长 0.1 s，按点序号线性均分」。位姿在 odom 缓冲里做线性插值 + 四元数 slerp。


## 调试

```bash
# 确认上游点云实际所处的坐标系（判断 cloud1 是否真在 Odin1 系）
ros2 topic echo --once /odin1_/cloud_slam --field header.frame_id

# 观察输出
ros2 topic hz /merged_cloud
ros2 topic echo --once /merged_cloud --field header
```
