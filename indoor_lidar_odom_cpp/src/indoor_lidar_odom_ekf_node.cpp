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

// PCL / ICP
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using std::placeholders::_1;

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
        Q_(0, 0) = 1e-4;
        Q_(1, 1) = 1e-4;
        Q_(2, 2) = 1e-4;
        Q_(3, 3) = 1e-2;
        Q_(4, 4) = 1e-2;

        R_.setZero();
        R_(0, 0) = 0.05 * 0.05;
        R_(1, 1) = 0.05 * 0.05;
        R_(2, 2) = (M_PI / 180.0) * (M_PI / 180.0);
    }

    void setInitialState(double x, double y, double yaw)
    {
        x_(0) = x;
        x_(1) = y;
        x_(2) = yaw;
        x_(3) = 0.0;
        x_(4) = 0.0;
    }

    const Eigen::Matrix<double, N, 1> &getState() const { return x_; }

    void predict(double wz, double ax, double ay, double dt)
    {
        if (dt <= 0.0)
            return;

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

        Eigen::Matrix<double, N, 1> x_pred;
        x_pred << x_new, y_new, yaw_new, vx_new, vy_new;

        Eigen::Matrix<double, N, N> F = Eigen::Matrix<double, N, N>::Identity();
        F(0, 3) = dt;
        F(1, 4) = dt;

        P_ = F * P_ * F.transpose() + Q_;
        x_ = x_pred;
    }

    void update(double x_meas, double y_meas, double yaw_meas)
    {
        Eigen::Matrix<double, 3, 1> z;
        z << x_meas, y_meas, yaw_meas;

        Eigen::Matrix<double, 3, 1> h;
        h << x_(0), x_(1), x_(2);

        Eigen::Matrix<double, 3, 1> y;
        y = z - h;
        y(2) = std::atan2(std::sin(y(2)), std::cos(y(2)));

        Eigen::Matrix<double, 3, N> H;
        H.setZero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 2) = 1.0;

        Eigen::Matrix<double, 3, 3> S = H * P_ * H.transpose() + R_;
        Eigen::Matrix<double, N, 3> K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;
        x_(2) = std::atan2(std::sin(x_(2)), std::cos(x_(2)));

        Eigen::Matrix<double, N, N> I = Eigen::Matrix<double, N, N>::Identity();
        P_ = (I - K * H) * P_;
    }

private:
    Eigen::Matrix<double, N, 1> x_;
    Eigen::Matrix<double, N, N> P_;
    Eigen::Matrix<double, N, N> Q_;
    Eigen::Matrix<double, 3, 3> R_;
};

// ===================== MAIN NODE =====================

class IndoorLidarOdomNode : public rclcpp::Node
{
public:
    IndoorLidarOdomNode()
        : Node("indoor_lidar_odom_ekf_node"),
          ekf_(),
          have_last_imu_time_(false),
          have_last_cloud_(false),
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

        RCLCPP_INFO(this->get_logger(), "IndoorLidarOdomNode with EKF + ICP started.");
    }

private:
    // ---------- Utility: yaw → quaternion ----------
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

    // ---------- EKF prediction from IMU ----------
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = msg->header.stamp;
        if (!have_last_imu_time_)
        {
            last_imu_time_ = current_time;
            have_last_imu_time_ = true;
            return;
        }

        double dt = (current_time - last_imu_time_).seconds();
        last_imu_time_ = current_time;
        if (dt <= 0.0)
            return;

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

    // ---------- ICP helper: estimate motion between two clouds ----------
    bool estimateMotionICP(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &prev_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &curr_cloud,
        double &dx_lidar,
        double &dy_lidar,
        double &dyaw_lidar)
    {
        if (!prev_cloud || !curr_cloud || prev_cloud->empty() || curr_cloud->empty())
        {
            return false;
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(curr_cloud); // scan at time k
        icp.setInputTarget(prev_cloud); // scan at time k-1

        icp.setMaximumIterations(30);
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        if (!icp.hasConverged())
        {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
            return false;
        }

        Eigen::Matrix4f T = icp.getFinalTransformation();
        // T maps curr -> prev: p_prev = T * p_curr

        dx_lidar = T(0, 3);
        dy_lidar = T(1, 3);
        double r00 = T(0, 0);
        double r10 = T(1, 0);
        dyaw_lidar = std::atan2(r10, r00);

        // Optional: you can inspect fitness for gating bad matches:
        // double fitness = icp.getFitnessScore();
        // RCLCPP_DEBUG(this->get_logger(), "ICP fitness=%.4f", fitness);

        return true;
    }

    // ---------- LiDAR callback: ICP + EKF update ----------
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

        // EKF predicted state at this time
        auto x_pred = ekf_.getState();
        double xk = x_pred(0);
        double yk = x_pred(1);
        double yawk = x_pred(2);

        bool have_meas = false;
        double x_meas = xk;
        double y_meas = yk;
        double yaw_meas = yawk;

        if (have_last_cloud_)
        {
            double dx_lidar = 0.0;
            double dy_lidar = 0.0;
            double dyaw_lidar = 0.0;

            bool ok = estimateMotionICP(last_cloud_, curr_cloud,
                                        dx_lidar, dy_lidar, dyaw_lidar);

            if (ok)
            {
                // Heuristic: if ICP says "mostly rotation, tiny translation",
                // treat it as pure rotation and ignore translation to avoid arcs.
                double rot_mag = std::fabs(dyaw_lidar);
                double trans_mag = std::hypot(dx_lidar, dy_lidar);
                if (rot_mag > 0.05 && trans_mag < 0.10)
                {
                    dx_lidar = 0.0;
                    dy_lidar = 0.0;
                }

                // ICP gives motion of environment in LiDAR frame (curr -> prev).
                // Robot motion is the opposite:
                double dx_robot_lidar = -dx_lidar;
                double dy_robot_lidar = -dy_lidar;

                // Rotate robot motion from LiDAR/base frame into odom using yaw at last scan
                double c = std::cos(last_scan_yaw_);
                double s = std::sin(last_scan_yaw_);

                double dx_global = c * dx_robot_lidar - s * dy_robot_lidar;
                double dy_global = s * dx_robot_lidar + c * dy_robot_lidar;

                x_meas = last_scan_x_ + dx_global;
                y_meas = last_scan_y_ + dy_global;
                yaw_meas = yawk; // keep yaw mainly from IMU/EKF prediction

                have_meas = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "ICP motion: dx_lidar=%.3f dy_lidar=%.3f -> dx_robot=%.3f dy_robot=%.3f -> meas=(%.3f, %.3f)",
                    dx_lidar, dy_lidar, dx_global, dy_global, x_meas, y_meas);
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "ICP motion estimate FAILED, skipping LiDAR update.");
            }
        }
        else
        {
            // First scan: just trust prediction as "measurement"
            have_meas = true;
            x_meas = xk;
            y_meas = yk;
            yaw_meas = yawk;
        }

        // 2) EKF update with LiDAR pose measurement
        if (have_meas)
        {
            ekf_.update(x_meas, y_meas, yaw_meas);
        }

        auto x_state = ekf_.getState();

        // 3) Publish odometry
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

        // 4) Store current cloud and state for next ICP
        last_cloud_ = curr_cloud;
        have_last_cloud_ = true;
        last_scan_x_ = x_state(0);
        last_scan_y_ = x_state(1);
        last_scan_yaw_ = x_state(2);
    }

    // ---------- Members ----------

    Ekf2D ekf_;

    // IMU timing
    rclcpp::Time last_imu_time_;
    bool have_last_imu_time_;

    // Last LiDAR cloud + pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
    bool have_last_cloud_;
    double last_scan_x_;
    double last_scan_y_;
    double last_scan_yaw_;

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
