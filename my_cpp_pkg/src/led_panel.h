#pragma once

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/led_panel_state.hpp"
#include "robot_interfaces/srv/set_led.hpp"

using LED_STATE = robot_interfaces::msg::LedPanelState;
using SET_LED = robot_interfaces::srv::SetLed;

class LedPanel : public rclcpp::Node
{
public:
    LedPanel();
private:
    rclcpp::Publisher<LED_STATE>::SharedPtr publisher_;
    rclcpp::Service<SET_LED>::SharedPtr server_;
    rclcpp::TimerBase::SharedPtr timer_;
    // bool ledState;
    LED_STATE ledState;
    void pubCallback();
    void setLedCallback(SET_LED::Request::SharedPtr request, SET_LED::Response::SharedPtr response);

};