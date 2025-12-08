#include "led_panel.h"
using namespace std::placeholders;
LedPanel::LedPanel() : Node("led_panel")
{
    
    this->declare_parameter("led1",false);
    this->declare_parameter("led2",false);
    this->declare_parameter("led3",false);

    publisher_ = this->create_publisher<LED_STATE>("led_state", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LedPanel::pubCallback, this));
    server_ = this->create_service<SET_LED>("/set_led", std::bind(&LedPanel::setLedCallback, this, _1, _2));
    ledState.led1 = this->get_parameter("led1").as_bool();
    ledState.led2 = this->get_parameter("led2").as_bool();
    ledState.led3 = this->get_parameter("led3").as_bool();
    RCLCPP_INFO(this->get_logger(), "Led Panel Node Started!!!!!!!");
}

void LedPanel::pubCallback()
{
    publisher_->publish(ledState);
}

void LedPanel::setLedCallback(SET_LED::Request::SharedPtr request, SET_LED::Response::SharedPtr response)
{
    ledState.led1 = request->led1;
    ledState.led2 = request->led2;
    ledState.led3 = request->led3;
    response->sucess = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanel>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}