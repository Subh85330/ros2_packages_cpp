#pragma once

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using INT64 = example_interfaces::msg::Int64;
using SET_BOOL = example_interfaces::srv::SetBool;

class NumberCounter : public rclcpp::Node
{

public:
    NumberCounter();

private:
    rclcpp::Publisher<INT64>::SharedPtr publisher_;
    rclcpp::Subscription<INT64>::SharedPtr subscriber_;
    rclcpp::Service<SET_BOOL>::SharedPtr server_;
    // rclcpp::TimerBase::SharedPtr subcriberTimer_;
    // rclcpp::TimerBase::SharedPtr pubTimer_;
    int counter_;
    
    void serviceCallback(SET_BOOL::Request::SharedPtr request,
                        SET_BOOL::Response::SharedPtr response);
    void subCallback(const INT64& msg);
};