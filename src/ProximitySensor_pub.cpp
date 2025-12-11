// src/ProximitySensor_pub.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp> // 0 또는 1을 보내기 위해 Int32 사용

class ProximitySensorPub : public rclcpp::Node
{
public:
    ProximitySensorPub() : Node("proximity_sensor_processor")
    {
        // 1. Subscriber: Raw 거리 데이터 구독
        raw_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "proximity_distance", 
            10, 
            std::bind(&ProximitySensorPub::topic_callback, this, std::placeholders::_1)
        );

        // 2. Publisher: 0 또는 1 상태 발행 (Int32 타입)
        // 토픽 이름: proximity_detection
        detection_pub_ = this->create_publisher<std_msgs::msg::Int32>("proximity_detection", 10);

        RCLCPP_INFO(this->get_logger(), "Proximity Processor Node Started. Threshold: 150mm");
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // 데이터가 비어있으면 무시
        if (msg->data.empty()) return;

        float current_distance = msg->data[0];
        std_msgs::msg::Int32 output_msg;

        if (current_distance > 0.0f && current_distance < 150.0f) {
            output_msg.data = 1; // 150mm 미만 (감지됨)
        } else {
            output_msg.data = 0; // 150mm 이상 (감지 안됨)
        }

        // 결과 발행
        detection_pub_->publish(output_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr raw_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr detection_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProximitySensorPub>());
    rclcpp::shutdown();
    return 0;
}