#pragma once

#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode();
private:
    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};