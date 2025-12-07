#include "cpp_mini.h"

MyNode::MyNode() : Node("my_first_node"), counter_(0)
{
    RCLCPP_INFO(this->get_logger(), "First Node Started");
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                    std::bind(&MyNode::timerCallback, this)
                                    );
}

void MyNode::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "First Node Running with counter %d", counter_);
    ++counter_;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}