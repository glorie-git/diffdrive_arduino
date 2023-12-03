#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "led_serial/arduino_comms.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "std_msgs/msg/string.hpp"


struct Config
{
  std::string device = "/dev/ttyACM0";
  int baud_rate = 57600;
  int timeout_ms = 1000;
};

class LedSerialNode : public rclcpp::Node
{
public:
    LedSerialNode();
    ~LedSerialNode();

    void on_init();
    void on_configure();
    void write(std::string val);

private:
    // void topic_callback(const std_msgs::msg::String & msg);
    void bool_callback (const std_msgs::msg::Bool & msg);

    void timer_callback();

    void topic_callback(const std_msgs::msg::String & msg); // const

    void robot_status_topic_callback(const std_msgs::msg::String & msg); // const
    void setColor ();

    void obj_detected_callback(const std_msgs::msg::Bool & msg);

    // void bool_callback(const std_msgs::msg::Bool & msg);

    Config cfg_;
    ArduinoComms comms_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obj_detected_sub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_subscriber_;
    int cmdArray_[2];
    int cmdIdx_; 

    bool stay_idle = true;

    std::chrono::steady_clock::time_point obj_detected_last_time;
    std::chrono::steady_clock::time_point follow_last_time;

    std_msgs::msg::String robot_status_;

    size_t count_;

};