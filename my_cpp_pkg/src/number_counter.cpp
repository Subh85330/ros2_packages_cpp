#include "number_counter.h"

using namespace std::placeholders;

NumberCounter::NumberCounter() : Node("number_counter"), counter_(0)
{
    publisher_ = this->create_publisher<INT64>("number_counter", 10);
    subscriber_ = this->create_subscription<INT64>("number", 10,
                                                   std::bind(&NumberCounter::subCallback, this, _1));

    server_ = this->create_service<SET_BOOL>("/reset_counter", 
                                            std::bind(&NumberCounter::serviceCallback, this, _1, _2));
}

void NumberCounter::serviceCallback(SET_BOOL::Request::SharedPtr request,
                                    SET_BOOL::Response::SharedPtr response)
{
    if(request->data)
    {
        counter_=0;
        response->success = true;
        return;
    }
    response->success = false;

}

void NumberCounter::subCallback(const INT64 &msg)
{
    counter_ += msg.data;
    auto newMsg = INT64();
    newMsg.data = counter_;
    publisher_->publish(newMsg);
    RCLCPP_INFO(this->get_logger(), "Publishing counter %d", counter_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}