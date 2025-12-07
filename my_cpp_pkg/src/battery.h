#pragma once
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/set_led.hpp"
#include <thread>

using SET_LED = robot_interfaces::srv::SetLed;

class BatteryManager : public rclcpp::Node
{
public:
    BatteryManager();
    void callService();

private:
    rclcpp::Client<SET_LED>::SharedPtr client_;
    bool led3State;
    void callback(rclcpp::Client<SET_LED>::SharedFuture future);
    rclcpp::TimerBase::SharedPtr timer_;
};