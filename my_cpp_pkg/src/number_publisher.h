#pragma once
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using INT64 = example_interfaces::msg::Int64;

class MyNumberPublisher : public rclcpp::Node
{
public:
    MyNumberPublisher();
    // ~MyNumberPublisher();
private:
    int number_;
    rclcpp::Publisher<INT64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();
};