// src/ProximitySensor_pub.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp> 

class ProximitySensorPub : public rclcpp::Node
{
public:
    ProximitySensorPub() : Node("proximity_sensor_processor")
    {
        // Declare parameters
        this->declare_parameter<std::string>("input_topic", "proximity_distance");
        this->declare_parameter<std::string>("output_topic", "proximity_detection");
        this->declare_parameter<double>("threshold_mm", 150.0);
        
        // Get parameters
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        threshold_mm_ = this->get_parameter("threshold_mm").as_double();
        
        // 1. Subscriber: subscribe proximity distance data
        proximity_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            input_topic, 
            10, 
            std::bind(&ProximitySensorPub::topic_callback, this, std::placeholders::_1)
        );

        // 2. Publisher: publish detection state (Int32: 0 or 1)
        detection_pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Proximity Processor Node Started. Threshold: %.1f mm", threshold_mm_);
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Ignore if message has no data
        if (msg->data.empty()) return;

        float current_distance = msg->data[0];
        std_msgs::msg::Int32 output_msg;

        if (current_distance > 0.0f && current_distance < threshold_mm_) {
            output_msg.data = 1; // detected: distance below threshold
        } else {
            output_msg.data = 0; // not detected: distance at or above threshold
        }

        // Publish result
        detection_pub_->publish(output_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr proximity_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr detection_pub_;
    double threshold_mm_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProximitySensorPub>());
    rclcpp::shutdown();
    return 0;
}