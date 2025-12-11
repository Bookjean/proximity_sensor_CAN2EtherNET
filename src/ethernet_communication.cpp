// src/ethernet_communication.cpp
#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <vector>
#include <std_msgs/msg/float32_multi_array.hpp>

class EthernetCommunication : public rclcpp::Node
{
public:
    EthernetCommunication() : Node("ethernet_communication")
    {
        // IP/Port parameters
        this->declare_parameter<std::string>("can_server_ip", "192.168.0.223");
        this->declare_parameter<int>("can_server_port", 4001);

        server_ip_ = this->get_parameter("can_server_ip").as_string();
        server_port_ = this->get_parameter("can_server_port").as_int();

        // Raw data publishers
        prox_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("proximity_distance", 10);
        tof_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("tof_distance", 10);
        filtered_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("filtered_distance", 10);

        // Initialize message data
        prox_msgs_.data = {0.0f, 0.0f};
        tof_msgs_.data = {0.0f, 0.0f};
        filtered_msgs_.data = {0.0f, 0.0f};

        // Try socket connection
        if (connect_server()) {
            receive_thread_ = std::thread(&EthernetCommunication::receive_loop, this);
        }

        // Publish raw data at 100 Hz via timer
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(10ms, [this]() {
            prox_pub_->publish(prox_msgs_);
            filtered_pub_->publish(filtered_msgs_);
            tof_pub_->publish(tof_msgs_);
        });
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

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr prox_pub_, tof_pub_, filtered_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float32MultiArray prox_msgs_, tof_msgs_, filtered_msgs_;

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
            RCLCPP_ERROR(this->get_logger(), "Invalid address");
            return false;
        }
        if (connect(socket_fd_, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed");
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

        // Big-endian parsing
        if (can_id == 0x41) {
            // Distance (Payload 0-3 -> Frame 6-9)
            uint32_t dist = (static_cast<uint32_t>(frame[6]) << 24) |
                            (static_cast<uint32_t>(frame[7]) << 16) |
                            (static_cast<uint32_t>(frame[8]) << 8)  |
                            (static_cast<uint32_t>(frame[9]) << 0);
            
            // Raw (Payload 4-7 -> Frame 10-13)
            uint32_t raw =  (static_cast<uint32_t>(frame[10]) << 24) |
                            (static_cast<uint32_t>(frame[11]) << 16) |
                            (static_cast<uint32_t>(frame[12]) << 8)  |
                            (static_cast<uint32_t>(frame[13]) << 0);

            prox_msgs_.data[0] = static_cast<float>(dist); // millimeters
            filtered_msgs_.data[0] = static_cast<float>(raw);

        } else if (can_id == 0x42) {
            // ToF (Payload 2-3 -> Frame 8-9)
            uint16_t tof = (static_cast<uint16_t>(frame[8]) << 8) |
                           (static_cast<uint16_t>(frame[9]) << 0);
            tof_msgs_.data[0] = static_cast<float>(tof);
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