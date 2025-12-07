#include "number_publisher.h"

MyNumberPublisher::MyNumberPublisher() : Node("number_publisher"), number_(1)
{
    publisher_ = this->create_publisher<INT64>("number", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                    std::bind(&MyNumberPublisher::timerCallback,
                                    this));
    RCLCPP_INFO(this->get_logger(), "number_publisher Node got Started!!!!!!!!");
}

void MyNumberPublisher::timerCallback()
{
    auto msg = INT64();
    msg.data = number_;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing %d", number_);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}