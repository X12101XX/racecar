//
// racecar_driver_node.cpp
//

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/racecar_driver.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <sstream>

class RacecarDriver : public rclcpp::Node
{
public:
RacecarDriver()
: Node("racecar_driver")
{
this->declare_parameter<std::string>("serial_port", "/dev/car");
this->declare_parameter<int>("baud_rate", 38400);
this->declare_parameter<std::string>("traffic_state_file", "/tmp/traffic_light_state.json");

std::string serial_port;
int baud_rate;

this->get_parameter("serial_port", serial_port);
this->get_parameter("baud_rate", baud_rate);
this->get_parameter("traffic_state_file", traffic_state_file_);

// Convert std::string to char*
char serial_port_char[serial_port.size() + 1];
std::strcpy(serial_port_char, serial_port.c_str());

art_racecar_init(baud_rate, serial_port_char);

subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
// "/cmd_vel", 1, std::bind(&RacecarDriver::TwistCallback, this, std::placeholders::_1));
"/car_cmd_vel", 1, std::bind(&RacecarDriver::TwistCallback, this, std::placeholders::_1));

teleop_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/teleop_cmd_vel", 1, std::bind(&RacecarDriver::teleop_TwistCallback, this, std::placeholders::_1));

traffic_light_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/light", 1, std::bind(&RacecarDriver::LightCallback, this, std::placeholders::_1));

traffic_state_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&RacecarDriver::PollTrafficStateFile, this));

command_keepalive_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&RacecarDriver::CommandKeepalive, this));

send_stop_cmd();

}

private:
void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
if (traffic_stop_)
{
send_stop_cmd();
return;
}

// double angle;
linear_x = twist->linear.x * 100 + 1500;
angle = 1500.0 - twist->angular.z * 970;
// angle = 1500.0 - twist->angular.z * 980;
RCLCPP_INFO(this->get_logger(), "线速度: %.2f    角速度: %.2f", linear_x, angle);

// angle = 2500.0 - twist->angular.z * 950.0 / 180.0;
send_cmd(static_cast<uint16_t>(linear_x), static_cast<uint16_t>(angle));
RecordDriveCommand();
// send_cmd(static_cast<uint16_t>(twist->linear.x), static_cast<uint16_t>(angle));
}


void teleop_TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
    if (traffic_stop_)
    {
        send_stop_cmd();
        return;
    }

    angle = 2183.0 - twist->angular.z * 970.0 / 180.0;
    linear_x = twist->linear.x;

    RCLCPP_INFO(this->get_logger(), "线速度: %.2f    角速度: %.2f", linear_x, angle);
    RCLCPP_INFO(this->get_logger(), "线速度: %.2f    角速度: %.2f", linear_x, angle);

    send_cmd(static_cast<uint16_t>(twist->linear.x), static_cast<uint16_t>(angle));
    RecordDriveCommand();
}

void LightCallback(const std_msgs::msg::String::SharedPtr msg)
{
    ApplyTrafficState(msg->data, "topic");
}

void PollTrafficStateFile()
{
    std::ifstream file(traffic_state_file_);
    if (!file.is_open())
    {
        return;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string payload = buffer.str();
    const std::string state = ExtractJsonString(payload, "state");
    if (state.empty() || state == "unknown")
    {
        ReleaseTrafficStopIfUnknownTimedOut();
        return;
    }

    ApplyTrafficState(state, "state_file");
}

std::string ExtractJsonString(const std::string &payload, const std::string &key)
{
    const std::string needle = "\"" + key + "\"";
    const size_t key_pos = payload.find(needle);
    if (key_pos == std::string::npos)
    {
        return "";
    }
    const size_t colon_pos = payload.find(':', key_pos + needle.size());
    if (colon_pos == std::string::npos)
    {
        return "";
    }
    const size_t start_quote = payload.find('"', colon_pos + 1);
    if (start_quote == std::string::npos)
    {
        return "";
    }
    const size_t end_quote = payload.find('"', start_quote + 1);
    if (end_quote == std::string::npos)
    {
        return "";
    }
    return payload.substr(start_quote + 1, end_quote - start_quote - 1);
}

void ApplyTrafficState(const std::string &state, const char *source)
{
    if (state == "red" || state == "yellow")
    {
        traffic_stop_ = true;
        last_traffic_stop_time_ = std::chrono::steady_clock::now();
        send_stop_cmd();
        if (state != last_reported_traffic_state_)
        {
            RCLCPP_WARN(this->get_logger(), "Traffic light %s received from %s, stopping immediately.", state.c_str(), source);
        }
    }
    else if (state == "green")
    {
        traffic_stop_ = false;
        if (state != last_reported_traffic_state_)
        {
            RCLCPP_INFO(this->get_logger(), "Green light received from %s, drive commands enabled.", source);
        }
    }
    last_reported_traffic_state_ = state;
}

void ReleaseTrafficStopIfUnknownTimedOut()
{
    if (!traffic_stop_)
    {
        last_reported_traffic_state_ = "unknown";
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - last_traffic_stop_time_;
    if (elapsed >= std::chrono::milliseconds(500))
    {
        traffic_stop_ = false;
        if (last_reported_traffic_state_ != "unknown")
        {
            RCLCPP_INFO(this->get_logger(), "Traffic light unknown timeout, drive commands enabled.");
        }
        last_reported_traffic_state_ = "unknown";
    }
}

void send_stop_cmd()
{
    linear_x = 1500.0;
    angle = 1500.0;
    send_cmd(static_cast<uint16_t>(linear_x), static_cast<uint16_t>(angle));
}

void RecordDriveCommand()
{
    have_recent_command_ = true;
    last_drive_command_time_ = std::chrono::steady_clock::now();
}

void CommandKeepalive()
{
    const auto now = std::chrono::steady_clock::now();
    const bool command_timed_out =
        !have_recent_command_ ||
        now - last_drive_command_time_ >= std::chrono::milliseconds(500);

    if (traffic_stop_ || command_timed_out)
    {
        send_stop_cmd();
    }
}




rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_light_subscription_;
rclcpp::TimerBase::SharedPtr traffic_state_timer_;
rclcpp::TimerBase::SharedPtr command_keepalive_timer_;
    std::string traffic_state_file_;
    std::string last_reported_traffic_state_;
    std::chrono::steady_clock::time_point last_traffic_stop_time_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_drive_command_time_ = std::chrono::steady_clock::now();
    bool traffic_stop_ = false;
    bool have_recent_command_ = false;
    double linear_x;            // 全局变量
    double angle;               // 全局变量
};

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<RacecarDriver>());
rclcpp::shutdown();
return 0;
}
