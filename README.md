# merge_cloud

把两路激光雷达（Odin1 + Livox）的点云标定对齐、运动补偿、裁剪切块后合成一路输出。

输入是 Odin1 的主点云和 Livox 的原始点云，输出是单帧的 `/merged_cloud`。节点由 Livox 帧驱动，
**输出频率等于 Livox 的帧率**。

## 目录结构

```
merge_cloud/
├── CMakeLists.txt
├── package.xml
├── config/merge_cloud.yaml       # 话题名、外参、盲区球参数
├── launch/merge_cloud.launch.py
└── src/merge_cloud_code.cpp      # 单文件实现，全部逻辑在 MergeCloudNode 类里
```

## 依赖与构建

ROS 2（开发环境为 Humble），`ament_cmake`。除标准消息包外依赖 `pcl_ros` / `pcl_conversions` 和
`livox_ros_driver2`。

```bash
# 需要先 source 一个包含 livox_ros_driver2 的 workspace
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

## 坐标系约定

**输出点云在 Odin1 系**（机器人系），不是世界系。因此：

- 裁剪盒和盲区球以**原点**为中心，天然跟随机器人
- `header.frame_id` 标为 `front_odin1`，与点云实际所处坐标系一致
- `header.stamp` 是扫描参考时刻，与该帧点位姿一致

## 已知限制

- **拼接是纯 append**，没有配准、没有重叠区去重。两雷达共同视野内的物体会出现两次，重叠区点密度翻倍，下游做聚类/平面拟合时注意阈值。
- **外参是手填的**，代码不会自动纠正偏差。误差 1° 在 5 m 处就是 8.7 cm 的双影。
- **时间配对没有容差校验**，也没有按时间淘汰。若 `cloud1_topic` 中断，最后一帧 cloud1 会被无限期拿去和后续每一帧合并。
- **cloud1 缓存只按帧数淘汰**（50 帧），不按时间。
- **两路点云的 intensity 语义可能不同**却塞进同一个字段：cloud1 若带 `rgb`/`rgba`/分离 RGB 字段会被转成 0~255 灰度，否则回退到自己的 `intensity`；cloud2 用 Livox 原始 `intensity`。两者语义不同时，下游按强度着色或阈值过滤会看到两片不一致。
- **每点时间字段按 uint32 解析**，只校验字段名不校验类型。若话题实际发布的是 float64，会抛异常；若是绝对时间而非偏移，会被静默钳位到 1 s。
- 回调都在默认的互斥回调组里，`MultiThreadedExecutor` 实际是串行的。

## 调试

```bash
# 确认上游点云实际所处的坐标系（判断 cloud1 是否真在 Odin1 系）
ros2 topic echo --once /odin1_/cloud_slam --field header.frame_id

# 观察输出
ros2 topic hz /merged_cloud
ros2 topic echo --once /merged_cloud --field header
```

RViz 里把 Fixed Frame 设为 `front_odin1`：两路点云应当紧贴机器人、随其移动且互相刚性咬合。
若 cloud2 扫出去钉在场景里不动，说明去畸变把点推到了世界系（本轮改动前的行为）。
