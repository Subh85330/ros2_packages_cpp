#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>

using std::placeholders::_1;

class IndoorLidarOdomNode : public rclcpp::Node
{
public:
  IndoorLidarOdomNode()
  : Node("indoor_lidar_odom_publisher_node"),
    x_(0.0),
    y_(0.0),
    yaw_(0.0),
    have_last_imu_time_(false),
    have_prev_cloud_(false)
  {
    // LiDAR
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/amr/lidar",
      rclcpp::SensorDataQoS(),
      std::bind(&IndoorLidarOdomNode::lidarCallback, this, _1));

    // IMU
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/amr/imu",
      rclcpp::SensorDataQoS(),
      std::bind(&IndoorLidarOdomNode::imuCallback, this, _1));

    // LiDAR odom output (for EKF)
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom_lidar",
      10);

    RCLCPP_INFO(this->get_logger(), "IndoorLidarOdomNode (LiDAR odom for EKF) started.");
  }

private:
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
  }

  // ------------ IMU: integrate yaw only ------------
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
    if (dt <= 0.0) {
      return;
    }

    double wz = msg->angular_velocity.z;
    last_imu_wz_ = wz;

    yaw_ += wz * dt;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

    RCLCPP_DEBUG(
      this->get_logger(),
      "IMU dt=%.4f, wz=%.5f -> yaw=%.3f rad",
      dt, wz, yaw_);
  }

  // ------------ ICP helper ------------
  bool estimateMotionICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & prev_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & curr_cloud,
    double & dx_lidar,
    double & dy_lidar,
    double & dyaw_lidar)
  {
    if (!prev_cloud || !curr_cloud || prev_cloud->empty() || curr_cloud->empty()) {
      return false;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(curr_cloud);   // scan at time k
    icp.setInputTarget(prev_cloud);   // scan at time k-1

    icp.setMaximumIterations(30);
    icp.setMaxCorrespondenceDistance(1.0);   // tune if needed
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);

    if (!icp.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
      return false;
    }

    Eigen::Matrix4f T = icp.getFinalTransformation();
    // T maps curr -> prev: p_prev = T * p_curr

    dx_lidar = T(0,3);
    dy_lidar = T(1,3);
    double r00 = T(0,0);
    double r10 = T(1,0);
    dyaw_lidar = std::atan2(r10, r00);

    double fitness = icp.getFitnessScore();
    RCLCPP_DEBUG(this->get_logger(), "ICP fitness=%.4f", fitness);

    return true;
  }

  // ------------ LiDAR: ICP + integrate translation ------------
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert PointCloud2 -> PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *curr_cloud);

    RCLCPP_INFO(
      this->get_logger(),
      "LiDAR frame at t=%u.%u -> points: %zu",
      msg->header.stamp.sec,
      msg->header.stamp.nanosec,
      curr_cloud->size());

    if (have_prev_cloud_) {
      double dx_lidar = 0.0;
      double dy_lidar = 0.0;
      double dyaw_lidar = 0.0;

      bool ok = estimateMotionICP(prev_cloud_, curr_cloud,
                                  dx_lidar, dy_lidar, dyaw_lidar);

      if (ok) {
        // Heuristic: if mostly rotation (spin in place), ignore tiny translation
        double rot_mag   = std::fabs(dyaw_lidar);
        double trans_mag = std::hypot(dx_lidar, dy_lidar);
        if (rot_mag > 0.05 && trans_mag < 0.10) {
          dx_lidar = 0.0;
          dy_lidar = 0.0;
        }

        // ICP gives env motion; robot motion is opposite
        double dx_robot_lidar = -dx_lidar;
        double dy_robot_lidar = -dy_lidar;

        // Rotate robot motion into odom frame using current yaw_
        double c = std::cos(yaw_);
        double s = std::sin(yaw_);

        double dx_global =  c * dx_robot_lidar - s * dy_robot_lidar;
        double dy_global =  s * dx_robot_lidar + c * dy_robot_lidar;

        x_ += dx_global;
        y_ += dy_global;

        RCLCPP_INFO(
          this->get_logger(),
          "ICP motion: dx_lidar=%.3f dy_lidar=%.3f -> dx=%.3f dy=%.3f -> x=%.3f y=%.3f yaw=%.3f",
          dx_lidar, dy_lidar, dx_global, dy_global, x_, y_, yaw_);
      } else {
        RCLCPP_WARN(this->get_logger(), "ICP motion estimate failed; skipping translation update.");
      }
    }

    // Publish LiDAR odometry (this is what EKF will fuse with IMU)
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "odom";       // global frame
    odom_msg.child_frame_id = "base_link";   // robot base frame

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = yawToQuaternion(yaw_);

    // Optional: we don't estimate velocity here, leave twist zero
    odom_pub_->publish(odom_msg);

    // Save for next ICP
    prev_cloud_ = curr_cloud;
    have_prev_cloud_ = true;
  }

  // ------------ Members ------------

  // Robot pose (LiDAR odom)
  double x_;
  double y_;
  double yaw_;

  // IMU timing
  rclcpp::Time last_imu_time_;
  bool have_last_imu_time_;
  double last_imu_wz_ = 0.0;

  // Previous LiDAR cloud for ICP
  pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
  bool have_prev_cloud_;

  // ROS
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
