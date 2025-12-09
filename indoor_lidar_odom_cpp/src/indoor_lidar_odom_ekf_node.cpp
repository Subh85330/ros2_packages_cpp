#include <cmath>
#include <vector>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include <Eigen/Dense>

using std::placeholders::_1;

struct Point2D
{
  float x;
  float y;
};

// ===================== EKF 2D CLASS =====================
// State x = [x, y, yaw, vx, vy]^T

class Ekf2D
{
public:
  static constexpr int N = 5;

  Ekf2D()
  {
    x_.setZero();
    P_.setIdentity();
    P_ *= 0.1;

    Q_.setZero();
    Q_(0,0) = 1e-4;
    Q_(1,1) = 1e-4;
    Q_(2,2) = 1e-4;
    Q_(3,3) = 1e-2;
    Q_(4,4) = 1e-2;

    R_.setZero();
    R_(0,0) = 0.05 * 0.05;
    R_(1,1) = 0.05 * 0.05;
    R_(2,2) = (M_PI/180.0)*(M_PI/180.0);
  }

  void setInitialState(double x, double y, double yaw)
  {
    x_(0) = x;
    x_(1) = y;
    x_(2) = yaw;
    x_(3) = 0.0;
    x_(4) = 0.0;
  }

  const Eigen::Matrix<double,N,1> & getState() const { return x_; }

  void predict(double wz, double ax, double ay, double dt)
  {
    if (dt <= 0.0) return;

    double x = x_(0);
    double y = x_(1);
    double yaw = x_(2);
    double vx = x_(3);
    double vy = x_(4);

    double yaw_new = yaw + wz * dt;
    yaw_new = std::atan2(std::sin(yaw_new), std::cos(yaw_new));

    double c = std::cos(yaw);
    double s = std::sin(yaw);
    double ax_w = c * ax - s * ay;
    double ay_w = s * ax + c * ay;

    double vx_new = vx + ax_w * dt;
    double vy_new = vy + ay_w * dt;

    double x_new = x + vx * dt + 0.5 * ax_w * dt * dt;
    double y_new = y + vy * dt + 0.5 * ay_w * dt * dt;

    Eigen::Matrix<double,N,1> x_pred;
    x_pred << x_new, y_new, yaw_new, vx_new, vy_new;

    Eigen::Matrix<double,N,N> F = Eigen::Matrix<double,N,N>::Identity();
    F(0,3) = dt;
    F(1,4) = dt;

    P_ = F * P_ * F.transpose() + Q_;
    x_ = x_pred;
  }

  void update(double x_meas, double y_meas, double yaw_meas)
  {
    Eigen::Matrix<double,3,1> z;
    z << x_meas, y_meas, yaw_meas;

    Eigen::Matrix<double,3,1> h;
    h << x_(0), x_(1), x_(2);

    Eigen::Matrix<double,3,1> y;
    y = z - h;
    y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));

    Eigen::Matrix<double,3,N> H;
    H.setZero();
    H(0,0) = 1.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;

    Eigen::Matrix<double,3,3> S = H * P_ * H.transpose() + R_;
    Eigen::Matrix<double,N,3> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));

    Eigen::Matrix<double,N,N> I = Eigen::Matrix<double,N,N>::Identity();
    P_ = (I - K * H) * P_;
  }

private:
  Eigen::Matrix<double,N,1> x_;
  Eigen::Matrix<double,N,N> P_;
  Eigen::Matrix<double,N,N> Q_;
  Eigen::Matrix<double,3,3> R_;
};

// ===================== MAIN NODE =====================

class IndoorLidarOdomNode : public rclcpp::Node
{
public:
  IndoorLidarOdomNode()
  : Node("indoor_lidar_odom_ekf_node"),
    ekf_(),
    have_last_imu_time_(false),
    have_last_scan_(false),
    last_scan_x_(0.0),
    last_scan_y_(0.0),
    last_scan_yaw_(0.0)
  {
    ekf_.setInitialState(0.0, 0.0, 0.0);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/amr/lidar",
      rclcpp::SensorDataQoS(),
      std::bind(&IndoorLidarOdomNode::lidarCallback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/amr/imu",
      rclcpp::SensorDataQoS(),
      std::bind(&IndoorLidarOdomNode::imuCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom_est",
      10);

    RCLCPP_INFO(this->get_logger(), "IndoorLidarOdomNode with EKF+ICP started.");
  }

private:
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
  }

  std::vector<Point2D> extract2DScan(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    std::vector<Point2D> scan2d;
    scan2d.reserve(cloud.width);

    const float max_abs_z = 0.5f;
    const int stride = 4;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    int idx = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++idx) {
      if (idx % stride != 0) {
        continue;
      }

      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (std::fabs(z) > max_abs_z) {
        continue;
      }

      Point2D p;
      p.x = x;
      p.y = y;
      scan2d.push_back(p);
    }

    return scan2d;
  }

  std::vector<Point2D> rotateScan(const std::vector<Point2D> & scan, double angle)
  {
    std::vector<Point2D> out;
    out.reserve(scan.size());

    double c = std::cos(angle);
    double s = std::sin(angle);

    for (const auto & p : scan) {
      Point2D r;
      r.x = c * p.x - s * p.y;
      r.y = s * p.x + c * p.y;
      out.push_back(r);
    }

    return out;
  }

  bool estimateTranslation2D(
    const std::vector<Point2D> & ref_scan,
    const std::vector<Point2D> & cur_scan,
    double max_corr_dist,
    double & dx_out,
    double & dy_out)
  {
    if (ref_scan.empty() || cur_scan.empty()) {
      return false;
    }

    const double max_corr_dist_sq = max_corr_dist * max_corr_dist;

    std::vector<double> dxs;
    std::vector<double> dys;
    dxs.reserve(cur_scan.size());
    dys.reserve(cur_scan.size());

    std::size_t target_pairs = 200;
    std::size_t step = 1;
    if (cur_scan.size() > target_pairs) {
      step = cur_scan.size() / target_pairs;
      if (step == 0) step = 1;
    }

    for (std::size_t i = 0; i < cur_scan.size(); i += step) {
      const auto & p = cur_scan[i];

      double best_dist_sq = std::numeric_limits<double>::max();
      const Point2D * best_q = nullptr;

      for (const auto & q : ref_scan) {
        double dx = q.x - p.x;
        double dy = q.y - p.y;
        double dist_sq = dx*dx + dy*dy;
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_q = &q;
        }
      }

      if (best_q && best_dist_sq <= max_corr_dist_sq) {
        dxs.push_back(best_q->x - p.x);
        dys.push_back(best_q->y - p.y);
      }
    }

    if (dxs.empty()) {
      return false;
    }

    double sum_dx = 0.0;
    double sum_dy = 0.0;
    for (std::size_t i = 0; i < dxs.size(); ++i) {
      sum_dx += dxs[i];
      sum_dy += dys[i];
    }

    dx_out = sum_dx / static_cast<double>(dxs.size());
    dy_out = sum_dy / static_cast<double>(dys.size());
    return true;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time current_time = msg->header.stamp;
    if (!have_last_imu_time_) {
      last_imu_time_ = current_time;
      have_last_imu_time_ = true;
      return;
    }

    double dt = (current_time - last_imu_time_).seconds();
    last_imu_time_ = current_time;
    if (dt <= 0.0) return;

    double wz = msg->angular_velocity.z;
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;

    ekf_.predict(wz, ax, ay, dt);

    auto x = ekf_.getState();
    RCLCPP_DEBUG(
      this->get_logger(),
      "IMU predict dt=%.4f wz=%.4f ax=%.3f ay=%.3f -> x=%.3f y=%.3f yaw=%.3f",
      dt, wz, ax, ay, x(0), x(1), x(2));
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<Point2D> scan2d = extract2DScan(*msg);

    RCLCPP_INFO(
      this->get_logger(),
      "LiDAR frame at t=%u.%u -> 2D scan size: %zu",
      msg->header.stamp.sec,
      msg->header.stamp.nanosec,
      scan2d.size());

    auto x_pred = ekf_.getState();
    double xk = x_pred(0);
    double yk = x_pred(1);
    double yawk = x_pred(2);

    bool have_meas = false;
    double x_meas = xk;
    double y_meas = yk;
    double yaw_meas = yawk;

    if (have_last_scan_) {
      double delta_yaw_imu = yawk - last_scan_yaw_;
      std::vector<Point2D> cur_in_prev_yaw = rotateScan(scan2d, -delta_yaw_imu);

      double dx_lidar = 0.0;
      double dy_lidar = 0.0;
      const double max_corr_dist = 0.5;

      bool ok = estimateTranslation2D(
        last_scan_,
        cur_in_prev_yaw,
        max_corr_dist,
        dx_lidar,
        dy_lidar);

      if (ok) {
        double c = std::cos(last_scan_yaw_);
        double s = std::sin(last_scan_yaw_);

        double dx_env = c * dx_lidar - s * dy_lidar;
        double dy_env = s * dx_lidar + c * dy_lidar;

        double dx_robot = -dx_env;
        double dy_robot = -dy_env;

        x_meas = last_scan_x_ + dx_robot;
        y_meas = last_scan_y_ + dy_robot;
        yaw_meas = yawk;  // keep yaw mainly from IMU

        have_meas = true;

        RCLCPP_INFO(
          this->get_logger(),
          "Scan match OK: dx_lidar=%.3f dy_lidar=%.3f -> dx_robot=%.3f dy_robot=%.3f -> meas=(%.3f, %.3f)",
          dx_lidar, dy_lidar, dx_robot, dy_robot, x_meas, y_meas);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Scan match FAILED for this frame, skipping LiDAR update.");
      }
    } else {
      have_meas = true;
      x_meas = xk;
      y_meas = yk;
      yaw_meas = yawk;
    }

    if (have_meas) {
      ekf_.update(x_meas, y_meas, yaw_meas);
    }

    auto x_state = ekf_.getState();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link_est";

    odom_msg.pose.pose.position.x = x_state(0);
    odom_msg.pose.pose.position.y = x_state(1);
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = yawToQuaternion(x_state(2));

    odom_msg.twist.twist.linear.x = x_state(3);
    odom_msg.twist.twist.linear.y = x_state(4);

    odom_pub_->publish(odom_msg);

    RCLCPP_INFO(
      this->get_logger(),
      "EKF /odom_est (x=%.3f, y=%.3f, yaw=%.3f)",
      x_state(0), x_state(1), x_state(2));

    last_scan_ = scan2d;
    have_last_scan_ = true;
    last_scan_x_ = x_state(0);
    last_scan_y_ = x_state(1);
    last_scan_yaw_ = x_state(2);
  }

  Ekf2D ekf_;

  rclcpp::Time last_imu_time_;
  bool have_last_imu_time_;

  std::vector<Point2D> last_scan_;
  bool have_last_scan_;
  double last_scan_x_;
  double last_scan_y_;
  double last_scan_yaw_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IndoorLidarOdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
