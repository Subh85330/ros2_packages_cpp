#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

struct Point2D
{
    float x;
    float y;
};

using std::placeholders::_1;

class IndoorLidarOdomNode : public rclcpp::Node
{
public:
    IndoorLidarOdomNode()
        : Node("indoor_lidar_odom_node"),
          x_(0.0),
          y_(0.0),
          yaw_(0.0)
    {
        // Subscriber to LiDAR point cloud
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/amr/lidar",
            rclcpp::SensorDataQoS(),
            std::bind(&IndoorLidarOdomNode::lidarCallback, this, _1));

        // Subscriber to IMU
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/amr/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&IndoorLidarOdomNode::imuCallback, this, _1));

        // Publisher for estimated odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom_est",
            10);

        RCLCPP_INFO(this->get_logger(), "IndoorLidarOdomNode started.");
    }

private:
    // Convert yaw (rad) to quaternion
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
        // Integrate angular velocity around z to update yaw_
        rclcpp::Time current_time = msg->header.stamp;

        if (!have_last_imu_time_)
        {
            last_imu_time_ = current_time;
            have_last_imu_time_ = true;
            return;
        }

        double dt = (current_time - last_imu_time_).seconds();
        last_imu_time_ = current_time;

        // Simple integration: yaw_new = yaw_old + wz * dt
        double wz = msg->angular_velocity.z;
        yaw_ += wz * dt;

        // Normalize yaw to [-pi, pi] to avoid unbounded growth
        yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));

        RCLCPP_DEBUG(
            this->get_logger(),
            "IMU dt=%.4f, wz=%.5f -> yaw=%.3f rad",
            dt, wz, yaw_);
    }

    void imuCallbackOld(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // For now just log some data to verify IMU subscription works.
        // Later you will integrate angular_velocity.z to update yaw_.
        RCLCPP_DEBUG(
            this->get_logger(),
            "IMU t=%u.%u ang_vel_z=%.5f",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            msg->angular_velocity.z);
    }

    void lidarCallbackOld(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // For now publish a dummy pose (0,0,0) in frame "odom_est".
        // Later, this is where you'll call your scan-matching algorithm and update x_, y_, yaw_.

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "odom";         // world / odom frame for estimate
        odom_msg.child_frame_id = "base_link_est"; // estimated robot body frame

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation = yawToQuaternion(yaw_);

        // (Optionally, you could fill twist or covariance here)

        odom_pub_->publish(odom_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Published /odom_est at t=%u.%u (x=%.3f, y=%.3f, yaw=%.3f)",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            x_, y_, yaw_);
    }

    void lidarCallbackOld1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1) Extract 2D scan from 3D cloud
        std::vector<Point2D> scan2d = extract2DScan(*msg);
        last_scan_ = scan2d;
        have_last_scan_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR frame at t=%u.%u -> 2D scan size: %zu points",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            scan2d.size());

        // 2) For now: publish odom using current (x_, y_, yaw_) as before
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "odom"; // using odom as global frame
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
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1) Extract 2D scan from 3D cloud (in lidar_nav frame)
        std::vector<Point2D> scan2d = extract2DScan(*msg);

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR frame at t=%u.%u -> 2D scan size: %zu points",
            msg->header.stamp.sec,
            msg->header.stamp.nanosec,
            scan2d.size());

        // 2) If we have a previous scan, estimate translation
        if (have_last_scan_)
        {
            // IMU yaw_ is global yaw in odom frame.
            // Use IMU to align orientations: rotate current scan back by Δyaw_imu
            double yaw_now = yaw_;
            double delta_yaw_imu = yaw_now - yaw_at_last_scan_;
            // bring current scan into previous yaw frame
            std::vector<Point2D> cur_in_prev_yaw = rotateScan(scan2d, -delta_yaw_imu);

            double dx_lidar = 0.0;
            double dy_lidar = 0.0;
            const double max_corr_dist = 0.5; // meters, tune if needed

            bool ok = estimateTranslation2D(
                last_scan_,      // reference scan (previous)
                cur_in_prev_yaw, // current scan in previous yaw frame
                max_corr_dist,
                dx_lidar,
                dy_lidar);

            if (ok)
            {
                // dx_lidar, dy_lidar are in lidar frame at previous yaw.
                // Transform this increment into global (odom) frame using yaw_at_last_scan_.
                double c = std::cos(yaw_at_last_scan_);
                double s = std::sin(yaw_at_last_scan_);

                double dx_global = -(c * dx_lidar - s * dy_lidar);
                double dy_global = -(s * dx_lidar + c * dy_lidar);

                x_ += dx_global;
                y_ += dy_global;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Scan match OK: dx_lidar=%.3f, dy_lidar=%.3f -> dx=%.3f, dy=%.3f -> x=%.3f, y=%.3f",
                    dx_lidar, dy_lidar, dx_global, dy_global, x_, y_);
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Scan match FAILED for this frame (no valid correspondences)");
            }
        }

        // 3) Store this scan and yaw for next time
        last_scan_ = scan2d;
        have_last_scan_ = true;
        yaw_at_last_scan_ = yaw_;

        // 4) Publish odometry with updated (x_, y_, yaw_)
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "odom"; // global frame
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
    }

    std::vector<Point2D> extract2DScan(const sensor_msgs::msg::PointCloud2 &cloud)
    {
        std::vector<Point2D> scan2d;
        scan2d.reserve(cloud.width); // rough guess

        // Parameters – you can tune these later
        const float max_abs_z = 0.5f; // keep points near horizontal plane
        const int stride = 4;         // keep every Nth point to downsample

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        int idx = 0;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++idx)
        {
            if (idx % stride != 0)
            {
                continue; // simple downsampling
            }

            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;

            // basic sanity & height filter
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                continue;
            }

            if (std::fabs(z) > max_abs_z)
            {
                continue; // ignore floor/ceiling/high stuff
            }

            Point2D p;
            p.x = x;
            p.y = y;
            scan2d.push_back(p);
        }

        return scan2d;
    }

    std::vector<Point2D> rotateScan(
        const std::vector<Point2D> &scan,
        double angle)
    {
        std::vector<Point2D> out;
        out.reserve(scan.size());

        const double c = std::cos(angle);
        const double s = std::sin(angle);

        for (const auto &p : scan)
        {
            Point2D r;
            r.x = c * p.x - s * p.y;
            r.y = s * p.x + c * p.y;
            out.push_back(r);
        }

        return out;
    }

    bool estimateTranslation2D(
        const std::vector<Point2D> &ref_scan,
        const std::vector<Point2D> &cur_scan,
        double max_corr_dist,
        double &dx_out,
        double &dy_out)
    {
        if (ref_scan.empty() || cur_scan.empty())
        {
            return false;
        }

        const double max_corr_dist_sq = max_corr_dist * max_corr_dist;

        std::vector<double> dxs;
        std::vector<double> dys;
        dxs.reserve(cur_scan.size());
        dys.reserve(cur_scan.size());

        // Downsample current scan if very dense
        const std::size_t target_pairs = 200;
        std::size_t step = 1;
        if (cur_scan.size() > target_pairs)
        {
            step = cur_scan.size() / target_pairs;
            if (step == 0)
            {
                step = 1;
            }
        }

        for (std::size_t i = 0; i < cur_scan.size(); i += step)
        {
            const auto &p = cur_scan[i];

            // Find nearest neighbor in ref_scan (brute force)
            double best_dist_sq = std::numeric_limits<double>::max();
            const Point2D *best_q = nullptr;

            for (const auto &q : ref_scan)
            {
                const double dx = q.x - p.x;
                const double dy = q.y - p.y;
                const double dist_sq = dx * dx + dy * dy;
                if (dist_sq < best_dist_sq)
                {
                    best_dist_sq = dist_sq;
                    best_q = &q;
                }
            }

            if (best_q && best_dist_sq <= max_corr_dist_sq)
            {
                dxs.push_back(best_q->x - p.x);
                dys.push_back(best_q->y - p.y);
            }
        }

        if (dxs.empty())
        {
            return false;
        }

        double sum_dx = 0.0;
        double sum_dy = 0.0;
        for (std::size_t i = 0; i < dxs.size(); ++i)
        {
            sum_dx += dxs[i];
            sum_dy += dys[i];
        }

        dx_out = sum_dx / static_cast<double>(dxs.size());
        dy_out = sum_dy / static_cast<double>(dys.size());
        return true;
    }

    // ##############################################################################################
    // #####################     Data Members    ####################################################
    // ##############################################################################################

    // State
    double x_;
    double y_;
    double yaw_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // IMU integration
    rclcpp::Time last_imu_time_;
    bool have_last_imu_time_ = false;

    // 2D scans (for now just store latest one)
    std::vector<Point2D> last_scan_;
    bool have_last_scan_ = false;
    double yaw_at_last_scan_ = 0.0; // yaw_ value when last_scan_ was captured
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IndoorLidarOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
