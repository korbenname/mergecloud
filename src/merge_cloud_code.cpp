#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <mutex>
#include <deque>

class MergeCloudNode : public rclcpp::Node
{
public:
  MergeCloudNode()
  : Node("merge_cloud_node")
  {
    this->declare_parameter<std::string>("cloud1_topic", "/odin1_/cloud_raw");
    this->declare_parameter<std::string>("cloud2_topic", "/livox/lidar/pointcloud");
    this->declare_parameter<std::string>("frame_id", "odin1");
    this->declare_parameter<std::string>("odom_topic", "/odin1_/odometry_highfreq");
    this->declare_parameter<double>("crop_half_size", 5.0);

    this->declare_parameter<double>("roll1", 0.0);
    this->declare_parameter<double>("pitch1", 0.0);
    this->declare_parameter<double>("yaw1", 0.0);
    this->declare_parameter<double>("tx1", 0.0);
    this->declare_parameter<double>("ty1", 0.0);
    this->declare_parameter<double>("tz1", 0.0);

    this->declare_parameter<double>("roll2", 0.0);
    this->declare_parameter<double>("pitch2", 0.0);
    this->declare_parameter<double>("yaw2", 0.0);
    this->declare_parameter<double>("tx2", 0.0);
    this->declare_parameter<double>("ty2", 0.0);
    this->declare_parameter<double>("tz2", 0.0);

    this->declare_parameter<bool>("blind_sphere_enable", false);
    this->declare_parameter<double>("blind_sphere_radius", 0.3);
    this->declare_parameter<double>("blind_sphere_cx", 0.0);
    this->declare_parameter<double>("blind_sphere_cy", 0.0);
    this->declare_parameter<double>("blind_sphere_cz", 0.0);
    this->declare_parameter<bool>("blind_sphere_follow_odom", true);

    this->get_parameter("cloud1_topic", cloud1_topic_);
    this->get_parameter("cloud2_topic", cloud2_topic_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("crop_half_size", crop_half_size_);

    this->get_parameter("roll1", roll1_);
    this->get_parameter("pitch1", pitch1_);
    this->get_parameter("yaw1", yaw1_);
    this->get_parameter("tx1", tx1_);
    this->get_parameter("ty1", ty1_);
    this->get_parameter("tz1", tz1_);

    this->get_parameter("roll2", roll2_);
    this->get_parameter("pitch2", pitch2_);
    this->get_parameter("yaw2", yaw2_);
    this->get_parameter("tx2", tx2_);
    this->get_parameter("ty2", ty2_);
    this->get_parameter("tz2", tz2_);

    this->get_parameter("blind_sphere_enable", blind_sphere_enable_);
    this->get_parameter("blind_sphere_radius", blind_sphere_radius_);
    this->get_parameter("blind_sphere_cx", blind_sphere_cx_);
    this->get_parameter("blind_sphere_cy", blind_sphere_cy_);
    this->get_parameter("blind_sphere_cz", blind_sphere_cz_);
    this->get_parameter("blind_sphere_follow_odom", blind_sphere_follow_odom_);

    if (!blind_sphere_follow_odom_) {
      RCLCPP_WARN(get_logger(),
                  "blind_sphere_follow_odom=false has no effect: the merged cloud is already in "
                  "the robot-centred Odin1 frame, so the blind sphere always follows the robot");
    }

    cloud1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud1_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MergeCloudNode::cloud1Callback, this, std::placeholders::_1));
    cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud2_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MergeCloudNode::cloud2Callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MergeCloudNode::odomCallback, this, std::placeholders::_1));

    merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_cloud", 10);
  }

private:
  struct OdomPose {
    rclcpp::Time stamp;
    Eigen::Vector3f pos;
    Eigen::Quaternionf rot;
  };

  void cloud1Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud1_buffer_.push_back(msg);
    while (cloud1_buffer_.size() > CLOUD1_BUFFER_MAX) {
      cloud1_buffer_.pop_front();
    }
  }

  void cloud2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_cloud2_ = msg;
    }
    mergeAndPublish();
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
  {
    OdomPose pose;
    pose.stamp = msg->header.stamp;
    pose.pos = Eigen::Vector3f(msg->pose.pose.position.x,
                               msg->pose.pose.position.y,
                               msg->pose.pose.position.z);
    const auto& q = msg->pose.pose.orientation;
    pose.rot = Eigen::Quaternionf(q.w, q.x, q.y, q.z);

    std::lock_guard<std::mutex> lock(mutex_);
    odom_buffer_.push_back(pose);
    while (odom_buffer_.size() > ODOM_BUFFER_MAX) {
      odom_buffer_.pop_front();
    }
  }

  void mergeAndPublish()
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud1_msg;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud2_msg;
    std::deque<OdomPose> odom_snapshot;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud1_snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cloud1_snapshot = cloud1_buffer_;
      cloud2_msg = latest_cloud2_;
      odom_snapshot = odom_buffer_;
    }

    if (!cloud2_msg) {
      return;
    }

    const rclcpp::Time cloud2_stamp = rclcpp::Time(cloud2_msg->header.stamp);

    // find cloud1 closest in time to cloud2
    if (!cloud1_snapshot.empty()) {
      cloud1_msg = cloud1_snapshot.front();
      double best_dt = std::abs((rclcpp::Time(cloud1_msg->header.stamp) - cloud2_stamp).seconds());
      for (const auto& c1 : cloud1_snapshot) {
        double dt = std::abs((rclcpp::Time(c1->header.stamp) - cloud2_stamp).seconds());
        if (dt < best_dt) {
          best_dt = dt;
          cloud1_msg = c1;
        }
      }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);

    if (cloud1_msg) {
      fillCloudXYZIFromMsg(*cloud1_msg, *cloud1, true);
    }
    if (cloud2_msg) {
      fillCloudXYZIFromMsg(*cloud2_msg, *cloud2, false);
    }

    // ---- static transforms first (move cloud2 to body frame) ----
    Eigen::Matrix4f cloud1_static = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f cloud2_static = Eigen::Matrix4f::Identity();
    setTransformMatrix(cloud1_static, static_cast<float>(roll1_), static_cast<float>(pitch1_),
                       static_cast<float>(yaw1_), static_cast<float>(tx1_),
                       static_cast<float>(ty1_), static_cast<float>(tz1_));
    setTransformMatrix(cloud2_static,
               static_cast<float>(roll2_), static_cast<float>(pitch2_), static_cast<float>(yaw2_),
               static_cast<float>(tx2_), static_cast<float>(ty2_), static_cast<float>(tz2_));

    if (cloud1_msg) {
      pcl::transformPointCloud(*cloud1, *cloud1, cloud1_static);
    }

    // ---- cloud2 chain: cloud2 frame -> Odin1 frame -> (deskew) -> output frame ----
    pcl::transformPointCloud(*cloud2, *cloud2, cloud2_static);

    // ---- deskew cloud2 using odometry ----
    // Motion compensation only: every point is brought back to the scan-start pose, so the
    // cloud stays in the Odin1 frame -- the frame this odometry is expressed in.
    deskewCloud(*cloud2_msg, *cloud2, cloud2_stamp, odom_snapshot);

    // Odin1 frame -> output frame
    pcl::transformPointCloud(*cloud2, *cloud2, cloud1_static);

    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>);
    if (cloud1_msg) {
      *merged = *cloud1 + *cloud2;
    } else {
      *merged = *cloud2;
    }

    // The merged cloud is in the Odin1 frame, which already travels with the robot, so the
    // crop box and the blind sphere are anchored at the origin rather than at the odometry
    // position (which would be a world coordinate and no longer meaningful here).
    const float half_size = static_cast<float>(crop_half_size_);
    const float cx = 0.0f;
    const float cy = 0.0f;

    const float bs_cx = static_cast<float>(blind_sphere_cx_);
    const float bs_cy = static_cast<float>(blind_sphere_cy_);
    const float bs_cz = static_cast<float>(blind_sphere_cz_);
    const float bs_r2 = blind_sphere_radius_ * blind_sphere_radius_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>);
    cropped->reserve(merged->size());
    for (const auto& pt : merged->points) {
      if (std::abs(pt.x - cx) > half_size || std::abs(pt.y - cy) > half_size) continue;
      if (blind_sphere_enable_) {
        float dx = pt.x - bs_cx;
        float dy = pt.y - bs_cy;
        float dz = pt.z - bs_cz;
        if (dx * dx + dy * dy + dz * dz <= bs_r2) continue;
      }
      cropped->points.push_back(pt);
    }
    cropped->width = static_cast<uint32_t>(cropped->points.size());
    cropped->height = 1;
    cropped->is_dense = merged->is_dense;
    merged.swap(cropped);

    sensor_msgs::msg::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged, merged_msg);
    merged_msg.header.frame_id = frame_id_;
    merged_msg.header.stamp = cloud2_stamp;
    merged_cloud_pub_->publish(merged_msg);
  }

  // ---- deskewing helpers ----

  bool interpolateOdom(const rclcpp::Time& stamp,
                       const std::deque<OdomPose>& buffer,
                       OdomPose& out) const
  {
    if (buffer.empty()) return false;
    if (buffer.size() == 1) {
      out = buffer.front();
      return true;
    }

    // find bounding poses: before <= stamp < after
    if (stamp <= buffer.front().stamp) {
      out = buffer.front();
      return true;
    }
    if (stamp >= buffer.back().stamp) {
      out = buffer.back();
      return true;
    }

    size_t after_idx = 0;
    for (size_t i = 0; i < buffer.size(); ++i) {
      if (buffer[i].stamp > stamp) {
        after_idx = i;
        break;
      }
    }
    size_t before_idx = after_idx - 1;

    const auto& before = buffer[before_idx];
    const auto& after = buffer[after_idx];

    double dt_total = (after.stamp - before.stamp).seconds();
    if (dt_total <= 0.0) {
      out = before;
      return true;
    }

    double alpha = (stamp - before.stamp).seconds() / dt_total;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;

    out.pos = (1.0f - alpha) * before.pos + alpha * after.pos;
    out.rot = before.rot.slerp(static_cast<float>(alpha), after.rot);
    out.stamp = stamp;
    return true;
  }

  bool findPerPointTimeField(const sensor_msgs::msg::PointCloud2& msg,
                             std::string& field_name) const
  {
    for (const auto& name : {"timestamp", "time", "t"}) {
      if (hasField(msg, name)) {
        field_name = name;
        return true;
      }
    }
    return false;
  }

  void deskewCloud(const sensor_msgs::msg::PointCloud2& msg,
                   pcl::PointCloud<pcl::PointXYZI>& cloud,
                   const rclcpp::Time& scan_start,
                   const std::deque<OdomPose>& odom_buffer)
  {
    if (odom_buffer.size() < 2 || cloud.empty()) return;

    // Reference pose: every point is compensated back to the scan start, which keeps the
    // result in the Odin1 frame instead of pushing it out into the odometry world frame.
    OdomPose odom_ref;
    if (!interpolateOdom(scan_start, odom_buffer, odom_ref)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "deskew: no odometry at scan start, skipping motion compensation");
      return;
    }
    const Eigen::Matrix3f r_ref_inv = odom_ref.rot.conjugate().toRotationMatrix();

    std::string time_field;
    const bool has_time = findPerPointTimeField(msg, time_field);

    if (has_time) {
      sensor_msgs::PointCloud2ConstIterator<uint32_t> it_t(msg, time_field);
      for (size_t i = 0; i < cloud.size(); ++i, ++it_t) {
        uint32_t raw = *it_t;
        double dt = static_cast<double>(raw) * 1e-9;
        if (dt < 0.0) dt = 0.0;
        if (dt > 1.0) dt = 1.0;

        rclcpp::Time pt_stamp(scan_start.nanoseconds() +
                              static_cast<int64_t>(dt * 1e9), scan_start.get_clock_type());

        OdomPose odom_pt;
        if (!interpolateOdom(pt_stamp, odom_buffer, odom_pt)) continue;

        // p_ref = T_world(t_ref)^-1 * T_world(t_point) * p
        applyRelative(cloud, i, r_ref_inv, odom_ref.pos, odom_pt);
      }
    } else {
      const size_t n = cloud.size();
      const double scan_duration = 0.1;
      for (size_t i = 0; i < n; ++i) {
        double dt = (n > 1) ? static_cast<double>(i) / static_cast<double>(n - 1) * scan_duration
                            : 0.0;

        rclcpp::Time pt_stamp(scan_start.nanoseconds() +
                              static_cast<int64_t>(dt * 1e9), scan_start.get_clock_type());

        OdomPose odom_pt;
        if (!interpolateOdom(pt_stamp, odom_buffer, odom_pt)) continue;

        applyRelative(cloud, i, r_ref_inv, odom_ref.pos, odom_pt);
      }
    }
  }

  // p_ref = T_ref^-1 * T_pt * p = R_ref^T*R_pt*p + R_ref^T*(t_pt - t_ref)
  void applyRelative(pcl::PointCloud<pcl::PointXYZI>& cloud, size_t idx,
                     const Eigen::Matrix3f& r_ref_inv,
                     const Eigen::Vector3f& ref_pos,
                     const OdomPose& pt_pose) const
  {
    const Eigen::Matrix3f R = r_ref_inv * pt_pose.rot.toRotationMatrix();
    const Eigen::Vector3f t = r_ref_inv * (pt_pose.pos - ref_pos);
    auto& pt = cloud[idx];
    const Eigen::Vector3f p(pt.x, pt.y, pt.z);
    const Eigen::Vector3f pr = R * p + t;
    pt.x = pr.x();
    pt.y = pr.y();
    pt.z = pr.z();
  }

  // ---- existing helpers ----

  void setTransformMatrix(Eigen::Matrix4f& transform,
                          float roll,
                          float pitch,
                          float yaw,
                          float tx,
                          float ty,
                          float tz)
  {
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
    transform.block<3, 3>(0, 0) = q.matrix();
    transform(0, 3) = tx;
    transform(1, 3) = ty;
    transform(2, 3) = tz;
  }

  bool hasField(const sensor_msgs::msg::PointCloud2& msg, const std::string& name) const
  {
    for (const auto& field : msg.fields) {
      if (field.name == name) {
        return true;
      }
    }
    return false;
  }

  void fillCloudXYZIFromMsg(const sensor_msgs::msg::PointCloud2& msg,
                            pcl::PointCloud<pcl::PointXYZI>& out,
                            bool prefer_rgb) const
  {
    const bool has_intensity = hasField(msg, "intensity");
    const bool has_rgb = hasField(msg, "rgb") || hasField(msg, "rgba");
    const bool has_split_rgb = hasField(msg, "r") && hasField(msg, "g") && hasField(msg, "b");
    const bool use_rgb = prefer_rgb && (has_rgb || has_split_rgb);

    out.clear();
    out.resize(static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height));

    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

    if (use_rgb && has_split_rgb) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> it_r(msg, "r");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> it_g(msg, "g");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> it_b(msg, "b");
      for (size_t i = 0; i < out.size(); ++i, ++it_x, ++it_y, ++it_z, ++it_r, ++it_g, ++it_b) {
        out.points[i].x = *it_x;
        out.points[i].y = *it_y;
        out.points[i].z = *it_z;
        out.points[i].intensity = 0.299f * (*it_r) + 0.587f * (*it_g) + 0.114f * (*it_b);
      }
      return;
    }

    if (use_rgb && has_rgb) {
      const std::string rgb_field = hasField(msg, "rgb") ? "rgb" : "rgba";
      sensor_msgs::PointCloud2ConstIterator<float> it_rgb(msg, rgb_field);
      for (size_t i = 0; i < out.size(); ++i, ++it_x, ++it_y, ++it_z, ++it_rgb) {
        uint32_t packed = 0u;
        const float rgb_value = *it_rgb;
        std::memcpy(&packed, &rgb_value, sizeof(float));
        const uint8_t r = static_cast<uint8_t>((packed >> 16) & 0xFF);
        const uint8_t g = static_cast<uint8_t>((packed >> 8) & 0xFF);
        const uint8_t b = static_cast<uint8_t>(packed & 0xFF);
        out.points[i].x = *it_x;
        out.points[i].y = *it_y;
        out.points[i].z = *it_z;
        out.points[i].intensity = 0.299f * r + 0.587f * g + 0.114f * b;
      }
      return;
    }

    if (has_intensity) {
      sensor_msgs::PointCloud2ConstIterator<float> it_i(msg, "intensity");
      for (size_t i = 0; i < out.size(); ++i, ++it_x, ++it_y, ++it_z, ++it_i) {
        out.points[i].x = *it_x;
        out.points[i].y = *it_y;
        out.points[i].z = *it_z;
        out.points[i].intensity = *it_i;
      }
      return;
    }

    for (size_t i = 0; i < out.size(); ++i, ++it_x, ++it_y, ++it_z) {
      out.points[i].x = *it_x;
      out.points[i].y = *it_y;
      out.points[i].z = *it_z;
      out.points[i].intensity = 0.0f;
    }
  }

  // ---- members ----

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;

  std::mutex mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud1_buffer_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud2_;
  std::deque<OdomPose> odom_buffer_;
  static constexpr size_t ODOM_BUFFER_MAX = 500;
  static constexpr size_t CLOUD1_BUFFER_MAX = 50;

  std::string cloud1_topic_;
  std::string cloud2_topic_;
  std::string frame_id_;
  std::string odom_topic_;
  double crop_half_size_ = 2.5;

  double roll1_ = 0.0;
  double pitch1_ = 0.0;
  double yaw1_ = 0.0;
  double tx1_ = 0.0;
  double ty1_ = 0.0;
  double tz1_ = 0.0;

  double roll2_ = 0.0;
  double pitch2_ = 0.0;
  double yaw2_ = 0.0;
  double tx2_ = 0.0;
  double ty2_ = 0.0;
  double tz2_ = 0.0;

  bool blind_sphere_enable_ = false;
  double blind_sphere_radius_ = 0.3;
  double blind_sphere_cx_ = 0.0;
  double blind_sphere_cy_ = 0.0;
  double blind_sphere_cz_ = 0.0;
  bool blind_sphere_follow_odom_ = true;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  auto node = std::make_shared<MergeCloudNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
