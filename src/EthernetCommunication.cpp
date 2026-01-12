// src/ethernet_communication.cpp
#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <vector>
#include <sensor_msgs/msg/range.hpp>

class EthernetCommunication : public rclcpp::Node
{
public:
    EthernetCommunication() : Node("ethernet_communication")
    {
        // Network parameters
        this->declare_parameter<std::string>("can_server_ip", "192.168.0.223");
        this->declare_parameter<int>("can_server_port", 4001);
        
        // Topic name parameters - 4 channels
        this->declare_parameter<std::string>("proximity_topic1", "proximity_distance1");
        this->declare_parameter<std::string>("proximity_topic2", "proximity_distance2");
        this->declare_parameter<std::string>("proximity_topic3", "proximity_distance3");
        this->declare_parameter<std::string>("proximity_topic4", "proximity_distance4");
        
        this->declare_parameter<std::string>("tof_topic1", "tof_distance1");
        this->declare_parameter<std::string>("tof_topic2", "tof_distance2");
        this->declare_parameter<std::string>("tof_topic3", "tof_distance3");
        this->declare_parameter<std::string>("tof_topic4", "tof_distance4");
        
        this->declare_parameter<std::string>("raw_topic1", "raw_distance1");
        this->declare_parameter<std::string>("raw_topic2", "raw_distance2");
        this->declare_parameter<std::string>("raw_topic3", "raw_distance3");
        this->declare_parameter<std::string>("raw_topic4", "raw_distance4");
        
        // Publish rate parameter
        this->declare_parameter<double>("publish_rate", 100.0);

        server_ip_ = this->get_parameter("can_server_ip").as_string();
        server_port_ = this->get_parameter("can_server_port").as_int();
        
        std::string proximity_topic1 = this->get_parameter("proximity_topic1").as_string();
        std::string proximity_topic2 = this->get_parameter("proximity_topic2").as_string();
        std::string proximity_topic3 = this->get_parameter("proximity_topic3").as_string();
        std::string proximity_topic4 = this->get_parameter("proximity_topic4").as_string();
        
        std::string tof_topic1 = this->get_parameter("tof_topic1").as_string();
        std::string tof_topic2 = this->get_parameter("tof_topic2").as_string();
        std::string tof_topic3 = this->get_parameter("tof_topic3").as_string();
        std::string tof_topic4 = this->get_parameter("tof_topic4").as_string();
        
        std::string raw_topic1 = this->get_parameter("raw_topic1").as_string();
        std::string raw_topic2 = this->get_parameter("raw_topic2").as_string();
        std::string raw_topic3 = this->get_parameter("raw_topic3").as_string();
        std::string raw_topic4 = this->get_parameter("raw_topic4").as_string();
        
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // Raw data publishers using sensor_msgs::Range (4 channels)
        prox_pub1_ = this->create_publisher<sensor_msgs::msg::Range>(proximity_topic1, 10);
        prox_pub2_ = this->create_publisher<sensor_msgs::msg::Range>(proximity_topic2, 10);
        prox_pub3_ = this->create_publisher<sensor_msgs::msg::Range>(proximity_topic3, 10);
        prox_pub4_ = this->create_publisher<sensor_msgs::msg::Range>(proximity_topic4, 10);
        
        tof_pub1_ = this->create_publisher<sensor_msgs::msg::Range>(tof_topic1, 10);
        tof_pub2_ = this->create_publisher<sensor_msgs::msg::Range>(tof_topic2, 10);
        tof_pub3_ = this->create_publisher<sensor_msgs::msg::Range>(tof_topic3, 10);
        tof_pub4_ = this->create_publisher<sensor_msgs::msg::Range>(tof_topic4, 10);
        
        raw_pub1_ = this->create_publisher<sensor_msgs::msg::Range>(raw_topic1, 10);
        raw_pub2_ = this->create_publisher<sensor_msgs::msg::Range>(raw_topic2, 10);
        raw_pub3_ = this->create_publisher<sensor_msgs::msg::Range>(raw_topic3, 10);
        raw_pub4_ = this->create_publisher<sensor_msgs::msg::Range>(raw_topic4, 10);

        // Initialize Range messages
        initialize_range_msg(prox_msgs1_, "proximity_sensor1_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 5000.0);
        initialize_range_msg(prox_msgs2_, "proximity_sensor2_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 5000.0);
        initialize_range_msg(prox_msgs3_, "proximity_sensor3_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 5000.0);
        initialize_range_msg(prox_msgs4_, "proximity_sensor4_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 5000.0);
        
        initialize_range_msg(tof_msgs1_, "tof_sensor1_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 8000.0);
        initialize_range_msg(tof_msgs2_, "tof_sensor2_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 8000.0);
        initialize_range_msg(tof_msgs3_, "tof_sensor3_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 8000.0);
        initialize_range_msg(tof_msgs4_, "tof_sensor4_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 8000.0);
        
        initialize_range_msg(raw_msgs1_, "raw_sensor1_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 10000.0);
        initialize_range_msg(raw_msgs2_, "raw_sensor2_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 10000.0);
        initialize_range_msg(raw_msgs3_, "raw_sensor3_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 10000.0);
        initialize_range_msg(raw_msgs4_, "raw_sensor4_frame", sensor_msgs::msg::Range::INFRARED, 0.0, 10000.0);

        // Try socket connection
        if (connect_server()) {
            receive_thread_ = std::thread(&EthernetCommunication::receive_loop, this);
        }

        // Publish raw data via timer
        using namespace std::chrono_literals;
        auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(period_ms, [this]() {
            prox_pub1_->publish(prox_msgs1_);
            prox_pub2_->publish(prox_msgs2_);
            prox_pub3_->publish(prox_msgs3_);
            prox_pub4_->publish(prox_msgs4_);
            
            tof_pub1_->publish(tof_msgs1_);
            tof_pub2_->publish(tof_msgs2_);
            tof_pub3_->publish(tof_msgs3_);
            tof_pub4_->publish(tof_msgs4_);
            
            raw_pub1_->publish(raw_msgs1_);
            raw_pub2_->publish(raw_msgs2_);
            raw_pub3_->publish(raw_msgs3_);
            raw_pub4_->publish(raw_msgs4_);
        });
        
        RCLCPP_INFO(this->get_logger(), "Publishers initialized: 4 channels (prox, tof, raw) at %.1f Hz", publish_rate);
    }

    ~EthernetCommunication()
    {
        running_ = false;
        if (receive_thread_.joinable()) receive_thread_.join();
        if (socket_fd_ >= 0) close(socket_fd_);
    }

private:
    std::string server_ip_;
    int server_port_;
    int socket_fd_ = -1;
    std::atomic<bool> running_{true};
    std::thread receive_thread_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr prox_pub1_, prox_pub2_, prox_pub3_, prox_pub4_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr tof_pub1_, tof_pub2_, tof_pub3_, tof_pub4_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr raw_pub1_, raw_pub2_, raw_pub3_, raw_pub4_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Range prox_msgs1_, prox_msgs2_, prox_msgs3_, prox_msgs4_;
    sensor_msgs::msg::Range tof_msgs1_, tof_msgs2_, tof_msgs3_, tof_msgs4_;
    sensor_msgs::msg::Range raw_msgs1_, raw_msgs2_, raw_msgs3_, raw_msgs4_;

    void initialize_range_msg(sensor_msgs::msg::Range& msg, const std::string& frame_id, 
                             uint8_t radiation_type, float min_range, float max_range)
    {
        msg.header.frame_id = frame_id;
        msg.radiation_type = radiation_type;
        msg.field_of_view = 0.1;  // radians, adjust based on sensor specs
        msg.min_range = min_range;  // millimeters
        msg.max_range = max_range;  // millimeters
        msg.range = 0.0;
    }

    bool connect_server()
    {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return false;
        }
        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(server_port_);
        if (inet_pton(AF_INET, server_ip_.c_str(), &serv_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address %s:%d", server_ip_.c_str(), server_port_);
            return false;
        }
        if (connect(socket_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed %s:%d", server_ip_.c_str(), server_port_ );
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Connected to %s:%d", server_ip_.c_str(), server_port_);
        return true;
    }

    void receive_loop()
    {
        uint8_t buffer[1024];
        std::vector<uint8_t> data_buffer;
        while (running_ && rclcpp::ok()) {
            ssize_t len = recv(socket_fd_, buffer, sizeof(buffer), 0);
            if (len <= 0) break;
            data_buffer.insert(data_buffer.end(), buffer, buffer + len);

            const size_t FRAME_SIZE = 14;
            while (data_buffer.size() >= FRAME_SIZE) {
                parse_frame(data_buffer.data());
                data_buffer.erase(data_buffer.begin(), data_buffer.begin() + FRAME_SIZE);
            }
        }
    }

    void parse_frame(const uint8_t* frame)
    {
        uint32_t can_id = frame[4];
        uint8_t dlc = frame[5];
        if (dlc > 8) return;

        // Big-endian parsing - 앞 4바이트 (Frame 6-9), 뒤 4바이트 (Frame 10-13)
        uint32_t data_front = (static_cast<uint32_t>(frame[6]) << 24) |
                              (static_cast<uint32_t>(frame[7]) << 16) |
                              (static_cast<uint32_t>(frame[8]) << 8)  |
                              (static_cast<uint32_t>(frame[9]) << 0);
        
        uint32_t data_back = (static_cast<uint32_t>(frame[10]) << 24) |
                             (static_cast<uint32_t>(frame[11]) << 16) |
                             (static_cast<uint32_t>(frame[12]) << 8)  |
                             (static_cast<uint32_t>(frame[13]) << 0);

        if (can_id == 0x41) {
            // 0x41: distance1 (앞 4바이트), tof1 (뒤 4바이트)
            prox_msgs1_.header.stamp = this->now();
            prox_msgs1_.range = static_cast<float>(data_front); // millimeters
            
            tof_msgs1_.header.stamp = this->now();
            tof_msgs1_.range = static_cast<float>(data_back); // millimeters

        } else if (can_id == 0x42) {
            // 0x42: distance2 (앞 4바이트), tof2 (뒤 4바이트)
            prox_msgs2_.header.stamp = this->now();
            prox_msgs2_.range = static_cast<float>(data_front); // millimeters
            
            tof_msgs2_.header.stamp = this->now();
            tof_msgs2_.range = static_cast<float>(data_back); // millimeters

        } else if (can_id == 0x43) {
            // 0x43: raw1 (앞 4바이트), raw2 (뒤 4바이트)
            raw_msgs1_.header.stamp = this->now();
            raw_msgs1_.range = static_cast<float>(data_front); // raw value
            
            raw_msgs2_.header.stamp = this->now();
            raw_msgs2_.range = static_cast<float>(data_back); // raw value

        } else if (can_id == 0x51) {
            // 0x51: distance3 (앞 4바이트), tof3 (뒤 4바이트)
            prox_msgs3_.header.stamp = this->now();
            prox_msgs3_.range = static_cast<float>(data_front); // millimeters
            
            tof_msgs3_.header.stamp = this->now();
            tof_msgs3_.range = static_cast<float>(data_back); // millimeters

        } else if (can_id == 0x52) {
            // 0x52: distance4 (앞 4바이트), tof4 (뒤 4바이트)
            prox_msgs4_.header.stamp = this->now();
            prox_msgs4_.range = static_cast<float>(data_front); // millimeters
            
            tof_msgs4_.header.stamp = this->now();
            tof_msgs4_.range = static_cast<float>(data_back); // millimeters

        } else if (can_id == 0x53) {
            // 0x53: raw3 (앞 4바이트), raw4 (뒤 4바이트)
            raw_msgs3_.header.stamp = this->now();
            raw_msgs3_.range = static_cast<float>(data_front); // raw value
            
            raw_msgs4_.header.stamp = this->now();
            raw_msgs4_.range = static_cast<float>(data_back); // raw value
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EthernetCommunication>());
    rclcpp::shutdown();
    return 0;
}
// CAN-to-Ethernet Communication Node