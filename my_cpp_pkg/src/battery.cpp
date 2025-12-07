#include "battery.h"

BatteryManager::BatteryManager() : Node("battery_manager"), led3State(true)
{
    client_ = this->create_client<SET_LED>("/set_led");
    timer_ = this->create_wall_timer(
        std::chrono::seconds(4), [this]()
        { callService(); });
    RCLCPP_INFO(this->get_logger(), "Battery Manager Node Started!!!!!!!");
}

void BatteryManager::callService()
{
    while (!client_->wait_for_service(std::chrono::seconds(4)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for server!!!!");
    }
    auto request = std::make_shared<SET_LED::Request>();
    request->led3 = led3State;
    client_->async_send_request(request,
                                std::bind(&BatteryManager::callback, this, std::placeholders::_1));
    led3State = led3State ? false : true;
}

void BatteryManager::callback(rclcpp::Client<SET_LED>::SharedFuture future)
{
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response %d", response->sucess);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}