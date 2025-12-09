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
    : Node("indoor_lidar_odom_node"),
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      have_last_imu_time_(false),
      have_prev_cloud_(false)
    {
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

        RCLCPP_INFO(this->get_logger(), "IndoorLidarOdomNode with PCL ICP started.");
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
        yaw_ += wz * dt;
        yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

        RCLCPP_DEBUG(
            this->get_logger(),
            "IMU dt=%.4f, wz=%.5f -> yaw=%.3f rad",
            dt, wz, yaw_);
    }

    bool estimateMotionICP(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &prev_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &curr_cloud,
        double &dx_lidar,
        double &dy_lidar,
        double &dyaw_lidar)
    {
        if (!prev_cloud || !curr_cloud || prev_cloud->empty() || curr_cloud->empty()) {
            return false;
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(curr_cloud);   // current scan
        icp.setInputTarget(prev_cloud);   // previous scan

        icp.setMaximumIterations(30);
        icp.setMaxCorrespondenceDistance(1.0);   // meters
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

        return true;
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1) Convert PointCloud2 -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *curr_cloud);

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR frame at t=%u.%u -> points: %zu",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            curr_cloud->size());

        double dx_lidar = 0.0;
        double dy_lidar = 0.0;
        double dyaw_lidar = 0.0;

        if (have_prev_cloud_) {
            bool ok = estimateMotionICP(prev_cloud_, curr_cloud, dx_lidar, dy_lidar, dyaw_lidar);

            if (ok) {
                // You can combine ICP yaw with IMU yaw if you want.
                // For now, we trust IMU yaw for orientation and let ICP provide translation only.
                // If you want, you can do: yaw_ += dyaw_lidar; then wrap.

                // dx_lidar, dy_lidar are in LiDAR frame at previous time.
                // Assuming LiDAR frame is aligned with base_link (x forward, y left),
                // transform this increment into odom frame using yaw_ at previous step.

                double c = std::cos(yaw_);
                double s = std::sin(yaw_);

                double dx_global =  -(c * dx_lidar - s * dy_lidar);
                double dy_global =  -(s * dx_lidar + c * dy_lidar);

                x_ += dx_global;
                y_ += dy_global;

                RCLCPP_INFO(
                    this->get_logger(),
                    "ICP motion: dx_lidar=%.3f dy_lidar=%.3f -> dx=%.3f dy=%.3f -> x=%.3f y=%.3f",
                    dx_lidar, dy_lidar, dx_global, dy_global, x_, y_);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "ICP motion estimate failed for this frame.");
            }
        }

        // 2) Publish odom
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link_est";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation = yawToQuaternion(yaw_);

        odom_pub_->publish(odom_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Published /odom_est (x=%.3f, y=%.3f, yaw=%.3f rad)",
            x_, y_, yaw_);

        // 3) Store current cloud as previous
        prev_cloud_ = curr_cloud;
        have_prev_cloud_ = true;
    }

    // State
    double x_;
    double y_;
    double yaw_;

    // IMU timing
    rclcpp::Time last_imu_time_;
    bool have_last_imu_time_;

    // Previous scan for ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    bool have_prev_cloud_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IndoorLidarOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
