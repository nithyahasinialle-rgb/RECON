#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rf_msgs/msg/wi_fi_rssi.hpp>
#include <Eigen/Dense>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

class EKFMultisensorFusionNode : public rclcpp::Node
{
public:
  EKFMultisensorFusionNode()
  : Node("ekf_fusion_node"),
    has_odom_(false),
    cmd_vx_(0.0),
    cmd_wz_(0.0),
    obstacle_count_(0),
    last_update_time_(this->now())
  {
    // State Vector x = [x, y, theta, vx, wz]^T (5x1)
    state_ = Eigen::VectorXd::Zero(5);
    state_(0) = 1.5; // Initial spawn x
    state_(1) = 1.5; // Initial spawn y
    cov_ = Eigen::MatrixXd::Identity(5, 5) * 0.05;

    trajectory_path_.header.frame_id = "odom";

    // Subscriptions
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&EKFMultisensorFusionNode::cmdVelCallback, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&EKFMultisensorFusionNode::imuCallback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&EKFMultisensorFusionNode::odomCallback, this, std::placeholders::_1));

    sub_tof_front_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/tof/front", 10,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) { tofCallback(msg, 0.0); });

    sub_tof_left_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/tof/left", 10,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) { tofCallback(msg, M_PI_2); });

    sub_tof_right_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/tof/right", 10,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) { tofCallback(msg, -M_PI_2); });

    sub_tof_rear_ = this->create_subscription<sensor_msgs::msg::Range>(
      "/tof/rear", 10,
      [this](const sensor_msgs::msg::Range::SharedPtr msg) { tofCallback(msg, M_PI); });

    sub_wifi_ = this->create_subscription<rf_msgs::msg::WiFiRSSI>(
      "/wifi/rssi", rclcpp::SensorDataQoS(),
      std::bind(&EKFMultisensorFusionNode::wifiCallback, this, std::placeholders::_1));

    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/rf_slam/pose", 10);
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/rf_slam/trajectory", 10);
    pub_ap_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rf_slam/map", 10);
    pub_obs_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rf_slam/obstacles", 10);
    pub_single_obs_ = this->create_publisher<visualization_msgs::msg::Marker>("/rf_slam/single_obstacles", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 20 Hz Real-Time Motion Prediction & Dynamic Mapping Loop
    timer_loop_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&EKFMultisensorFusionNode::updateLoop, this));

    RCLCPP_INFO(this->get_logger(), "C++ EKF Multisensor Fusion Engine Running on Jetson Orin Nano");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    cmd_vx_ = msg->linear.x;
    cmd_wz_ = msg->angular.z;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    has_odom_ = true;
    state_(3) = msg->twist.twist.linear.x;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    state_(2) = yaw;
    state_(4) = msg->angular_velocity.z;
  }

  void updateLoop()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();

    if (dt < 0.0 || dt > 0.5) {
      dt = 0.05;
    }

    double vx = (std::abs(cmd_vx_) > 1e-3) ? cmd_vx_ : (has_odom_ ? state_(3) : 0.0);
    double wz = (std::abs(cmd_wz_) > 1e-3) ? cmd_wz_ : (state_(4) != 0.0 ? state_(4) : 0.0);

    state_(0) += vx * std::cos(state_(2)) * dt;
    state_(1) += vx * std::sin(state_(2)) * dt;
    state_(2) += wz * dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(5, 5);
    Q(0, 0) = 0.02 * dt;
    Q(1, 1) = 0.02 * dt;
    Q(2, 2) = 0.005 * dt;
    cov_ = cov_ + Q;

    last_update_time_ = current_time;

    publishPoseAndTF(current_time);
    publishAPMarkers(current_time);

    if (!obstacle_markers_.markers.empty()) {
      for (auto & m : obstacle_markers_.markers) {
        m.header.stamp = current_time;
      }
      pub_obs_markers_->publish(obstacle_markers_);
      publishSingleObstacles(current_time);
    }
  }

  void tofCallback(const sensor_msgs::msg::Range::SharedPtr msg, double sensor_angle_offset)
  {
    if (msg->range < msg->min_range || msg->range > msg->max_range) return;

    double ray_heading = state_(2) + sensor_angle_offset;
    double rx = state_(0);
    double ry = state_(1);

    double dx = std::cos(ray_heading);
    double dy = std::sin(ray_heading);

    double expected_range = 2.0;
    if (std::abs(dx) > 1e-3) {
      double d_wall = (dx > 0) ? (10.0 - rx) / dx : -rx / dx;
      if (d_wall > 0 && d_wall < expected_range) expected_range = d_wall;
    }
    if (std::abs(dy) > 1e-3) {
      double d_wall = (dy > 0) ? (10.0 - ry) / dy : -ry / dy;
      if (d_wall > 0 && d_wall < expected_range) expected_range = d_wall;
    }

    double residual = msg->range - expected_range;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 5);
    H(0, 0) = -dx;
    H(0, 1) = -dy;

    double R = 0.008;
    double S = (H * cov_ * H.transpose())(0, 0) + R;
    Eigen::VectorXd K = cov_ * H.transpose() / S;

    state_ = state_ + K * residual;
    cov_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * cov_;

    if (msg->range < 1.99) {
      double obs_x = rx + msg->range * std::cos(ray_heading);
      double obs_y = ry + msg->range * std::sin(ray_heading);
      addObstacleToMap(obs_x, obs_y);
    }
  }

  void wifiCallback(const rf_msgs::msg::WiFiRSSI::SharedPtr msg)
  {
    double ax = msg->ap_location.x;
    double ay = msg->ap_location.y;
    double rx = state_(0);
    double ry = state_(1);

    double dist_est = std::sqrt(std::pow(rx - ax, 2) + std::pow(ry - ay, 2));
    dist_est = std::max(0.1, dist_est);

    double dist_meas = std::pow(10.0, (msg->rssi_at_1m - msg->rssi) / (10.0 * msg->path_loss_exponent));
    double residual = dist_meas - dist_est;

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 5);
    H(0, 0) = (rx - ax) / dist_est;
    H(0, 1) = (ry - ay) / dist_est;

    double R = 1.5;
    double S = (H * cov_ * H.transpose())(0, 0) + R;
    Eigen::VectorXd K = cov_ * H.transpose() / S;

    state_ = state_ + K * residual;
    cov_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * cov_;
  }

  void publishPoseAndTF(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "odom";
    pose.pose.position.x = state_(0);
    pose.pose.position.y = state_(1);
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state_(2));
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    pub_pose_->publish(pose);

    trajectory_path_.header.stamp = stamp;
    if (trajectory_path_.poses.empty() ||
        std::hypot(trajectory_path_.poses.back().pose.position.x - state_(0),
                   trajectory_path_.poses.back().pose.position.y - state_(1)) > 0.05)
    {
      trajectory_path_.poses.push_back(pose);
      pub_path_->publish(trajectory_path_);
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = state_(0);
    tf_msg.transform.translation.y = state_(1);
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishAPMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray markers;

    // 1. Realistic 3D Conference Tables (Wood Top & Metallic Finish)
    struct Table { double x, y, sx, sy, r, g, b; };
    std::vector<Table> tables = {
      {3.5, 3.5, 1.6, 0.9, 0.55f, 0.35f, 0.20f}, // Conference Table 1 (Oak Wood)
      {6.5, 6.5, 1.6, 0.9, 0.25f, 0.25f, 0.30f}  // Conference Table 2 (Dark Walnut)
    };
    for (size_t i = 0; i < tables.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = "odom";
      m.ns = "office_tables";
      m.id = 10 + i + 1;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = tables[i].x;
      m.pose.position.y = tables[i].y;
      m.pose.position.z = 0.375;
      m.scale.x = tables[i].sx; m.scale.y = tables[i].sy; m.scale.z = 0.75;
      m.color.r = tables[i].r; m.color.g = tables[i].g; m.color.b = tables[i].b; m.color.a = 0.90f;
      markers.markers.push_back(m);
    }

    // 2. Storage Cabinet at (1.2, 7.5) & Bookshelf Unit at (8.8, 2.5)
    visualization_msgs::msg::Marker cab;
    cab.header.stamp = stamp;
    cab.header.frame_id = "odom";
    cab.ns = "storage_cabinets";
    cab.id = 30;
    cab.type = visualization_msgs::msg::Marker::CUBE;
    cab.action = visualization_msgs::msg::Marker::ADD;
    cab.pose.position.x = 1.2; cab.pose.position.y = 7.5; cab.pose.position.z = 0.70;
    cab.scale.x = 0.6; cab.scale.y = 1.4; cab.scale.z = 1.4;
    cab.color.r = 0.30f; cab.color.g = 0.35f; cab.color.b = 0.40f; cab.color.a = 0.90f;
    markers.markers.push_back(cab);

    visualization_msgs::msg::Marker shelf;
    shelf.header.stamp = stamp;
    shelf.header.frame_id = "odom";
    shelf.ns = "bookshelves";
    shelf.id = 31;
    shelf.type = visualization_msgs::msg::Marker::CUBE;
    shelf.action = visualization_msgs::msg::Marker::ADD;
    shelf.pose.position.x = 8.8; shelf.pose.position.y = 2.5; shelf.pose.position.z = 0.80;
    shelf.scale.x = 0.5; shelf.scale.y = 1.2; shelf.scale.z = 1.6;
    shelf.color.r = 0.45f; shelf.color.g = 0.30f; shelf.color.b = 0.20f; shelf.color.a = 0.90f;
    markers.markers.push_back(shelf);

    // 3. 3 Wi-Fi Access Point Telemetry Beacons
    std::vector<std::pair<double, double>> aps = {{0.5, 0.5}, {9.5, 0.5}, {5.0, 9.5}};
    for (size_t i = 0; i < aps.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = "odom";
      m.ns = "wifi_access_points";
      m.id = i + 1;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = aps[i].first;
      m.pose.position.y = aps[i].second;
      m.pose.position.z = 0.75;
      m.scale.x = 0.25; m.scale.y = 0.25; m.scale.z = 1.5;
      m.color.r = 1.0f; m.color.g = 0.45f; m.color.b = 0.0f; m.color.a = 0.95f;
      markers.markers.push_back(m);
    }

    pub_ap_markers_->publish(markers);
  }

  void addObstacleToMap(double x, double y)
  {
    for (const auto & m : obstacle_markers_.markers) {
      if (std::hypot(m.pose.position.x - x, m.pose.position.y - y) < 0.05) {
        return;
      }
    }

    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = "odom";
    m.ns = "tof_mapped_obstacles";
    m.id = ++obstacle_count_;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.08;
    m.scale.x = 0.10; m.scale.y = 0.10; m.scale.z = 0.10;
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.95f;
    
    obstacle_markers_.markers.push_back(m);
  }

  void publishSingleObstacles(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = "odom";
    m.ns = "single_obstacles";
    m.id = 777;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.10; m.scale.y = 0.10; m.scale.z = 0.10;
    m.color.r = 1.0f; m.color.g = 0.0f; m.color.b = 0.0f; m.color.a = 0.95f;

    for (const auto & obs : obstacle_markers_.markers) {
      geometry_msgs::msg::Point p;
      p.x = obs.pose.position.x;
      p.y = obs.pose.position.y;
      p.z = obs.pose.position.z;
      m.points.push_back(p);
    }
    pub_single_obs_->publish(m);
  }

  bool has_odom_;
  double cmd_vx_;
  double cmd_wz_;
  int obstacle_count_;
  rclcpp::Time last_update_time_;

  Eigen::VectorXd state_;
  Eigen::MatrixXd cov_;

  nav_msgs::msg::Path trajectory_path_;
  visualization_msgs::msg::MarkerArray obstacle_markers_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_tof_front_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_tof_left_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_tof_right_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_tof_rear_;
  rclcpp::Subscription<rf_msgs::msg::WiFiRSSI>::SharedPtr sub_wifi_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_ap_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obs_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_single_obs_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_loop_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EKFMultisensorFusionNode>();
  rclcpp::spin(node);
  return 0;
}
