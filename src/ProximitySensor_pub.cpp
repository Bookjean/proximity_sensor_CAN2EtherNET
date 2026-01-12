// src/ProximitySensor_pub.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp> 

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
        
        // 1. Subscriber: subscribe proximity distance data (sensor_msgs::Range)
        proximity_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            input_topic, 
            10, 
            std::bind(&ProximitySensorPub::topic_callback, this, std::placeholders::_1)
        );

        // 2. Publisher: publish detection state (Bool: true/false)
        detection_pub_ = this->create_publisher<std_msgs::msg::Bool>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Proximity Processor Node Started. Threshold: %.1f mm", threshold_mm_);
    }

private:
    void topic_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        float current_distance_mm = msg->range * 1000.0; // Convert meters to mm
        std_msgs::msg::Bool output_msg;

        if (current_distance_mm > 0.0f && current_distance_mm < threshold_mm_) {
            output_msg.data = true; // detected: distance below threshold
        } else {
            output_msg.data = false; // not detected: distance at or above threshold
        }

        // Publish result
        detection_pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr proximity_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detection_pub_;
    double threshold_mm_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProximitySensorPub>());
    rclcpp::shutdown();
    return 0;
}