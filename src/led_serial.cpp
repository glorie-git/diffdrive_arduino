#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "led_serial/led_serial.hpp"
#include <ctime>
#include <chrono>
// #include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

LedSerialNode::LedSerialNode()
    : Node("led_serial_node")
{

    RCLCPP_INFO(get_logger(), "LED SERIAL NODE STARTED");
    // Initialize the Arduino communication
    comms_.connect("/dev/ttyACM0", 57600, 1000);  // Replace with your actual serial device and baud rate
    // comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);  // Replace with your actual serial device and baud rate

    on_init();
    on_configure();

    this->cmdIdx_ = 1;
    cmdArray_[0] = this->cmdIdx_;
    cmdArray_[1] = 3;

    stay_idle = true;

    this->subscription_ = this->create_subscription<std_msgs::msg::String>(
    "robot_status", 1, std::bind(&LedSerialNode::robot_status_topic_callback, this, _1));

    robot_status_pub_ = this->create_publisher<std_msgs::msg::String>("robot_status", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&LedSerialNode::timer_callback, this));

    bool_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "lost", 10, std::bind(&LedSerialNode::bool_callback, this, std::placeholders::_1));

    obj_detected_sub = this->create_subscription<std_msgs::msg::Bool>(
      "object_detected", 10, std::bind(&LedSerialNode::obj_detected_callback, this, std::placeholders::_1));

    robot_status_.data = "idle"; // start off in ideal mode "ball_tracker off"
}

LedSerialNode::~LedSerialNode(){}

void LedSerialNode::on_init(){
    cfg_.device = "/dev/ttyACM0";
    cfg_.baud_rate = 57600;
}

void LedSerialNode::robot_status_topic_callback(const std_msgs::msg::String & msg) // const
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    std::string status = msg.data.c_str();
    int error = 1;

    if ( status.compare("standby") == 0 ) {
        // SOLID ORANGE
        robot_status_.data = msg.data.c_str();
        // cmdArray_[0] = 1;
        // cmdArray_[1] = 1;
    } else if ( status.compare("thumbs_up") == 0 ) {
        // SOLID GREEN
        stay_idle = false;
        robot_status_.data = msg.data.c_str();
    } else if ( status.compare("idle") == 0 ) {
        // FLASHING ORANGE
        stay_idle = true;
        robot_status_.data = msg.data.c_str();
        // cmdArray_[0] = 1;
        // cmdArray_[1] = 3;
    } else if ( status.compare("following") == 0 ) {
        // FLASHING GREEN
        robot_status_.data = msg.data.c_str();
        // cmdArray_[0] = 2;
        // cmdArray_[1] = 3;
    }  else if ( status.compare("error") == 0 ) {
        // FLASHING RED
        robot_status_.data = msg.data.c_str();

    } else if (status.compare("obj_detected") == 0) {
        // FLASHING BLUE
        robot_status_.data = msg.data.c_str();

    } else {
        error = -1;
    }

    if (error == 1) {
        setColor();
    } else {
        RCLCPP_INFO(this->get_logger(), "Invalid robot_status: '%s'", msg.data.c_str());
    }
}

void LedSerialNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    std::stringstream ss;
    if (robot_status_.data.compare("obj_detected") == 0){
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> timeElapsed = currentTime - obj_detected_last_time;

        // std::cout << "Last object time: " << timeElapsed.count() << "seconds" << std::endl;
        if (timeElapsed.count() > 0.5) {
            if (stay_idle) {
                robot_status_.data = "idle";
            } else {
                robot_status_.data = "standby";
            }
            robot_status_pub_->publish(robot_status_);
            setColor();
        }
    } else if (robot_status_.data.compare("following") == 0 ) {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> timeElapsed = currentTime - follow_last_time;

        // std::cout << "Last follow time: " << timeElapsed.count() << "seconds" << std::endl;
        if (timeElapsed.count() > 0.5) {

            if (stay_idle) {
                robot_status_.data = "idle";
            } else {
                robot_status_.data = "standby";
            }

            robot_status_pub_->publish(robot_status_);
            setColor();
        }
    }
    ss << "l " << cmdArray_[count_++%2];
    message.data = ss.str(); 
    // RCLCPP_INFO(this->get_logger(), "Writing: '%s'", message.data.c_str());  // disable print to screen (to safe battery?)
    write(ss.str());
}

void LedSerialNode::setColor () {

    if ( robot_status_.data.compare("standby") == 0 ) {
        // SOLID ORANGE
        cmdArray_[0] = 1;
        cmdArray_[1] = 1;
    } else if ( robot_status_.data.compare("thumbs_up") == 0 ) {
        // SOLID GREEN
        cmdArray_[0] = 2;
        cmdArray_[1] = 2;
        
    } else if ( robot_status_.data.compare("idle") == 0 ) {
        // FLASHING ORANGE
        cmdArray_[0] = 1;
        cmdArray_[1] = 3;
    } else if ( robot_status_.data.compare("following") == 0 ) {
        // FLASHING GREEN
        cmdArray_[0] = 2;
        cmdArray_[1] = 3;
    }  else if ( robot_status_.data.compare("error") == 0 ) {
        // FLASHING RED
        cmdArray_[0] = 0;
        cmdArray_[1] = 3;    	
    }  else if (robot_status_.data.compare("obj_detected") == 0) {
        // FLASHING BLUE
        cmdArray_[0] = 4;
        cmdArray_[1] = 3; 
    }

}

void LedSerialNode::bool_callback(const std_msgs::msg::Bool & msg)
{   
    if (msg.data == true) {
        // SOLID ORANGE
        // RCLCPP_INFO(get_logger(), "Target is: lost"); // disable print to screen (to safe battery?)
        robot_status_.data = "standby";
    }
    else if (msg.data == false){
        // FLASHING GREEN
        // RCLCPP_INFO(get_logger(), "Target is: found!"); // disable print to screen (to safe battery?)
        robot_status_.data = "following";
        follow_last_time = std::chrono::steady_clock::now();
    }
    robot_status_pub_->publish(robot_status_);
    setColor();
}

void LedSerialNode::obj_detected_callback(const std_msgs::msg::Bool & msg)
{   
    if (msg.data == true) {
        // SOLID ORANGE
        // RCLCPP_INFO(get_logger(), "Target is: lost"); // disable print to screen (to safe battery?)
        robot_status_.data = "obj_detected";

    
    // else {
    //     robot_status_.data = "standby";
    // }
    obj_detected_last_time = std::chrono::steady_clock::now();

    robot_status_pub_->publish(robot_status_);
    setColor();
    }
}

void LedSerialNode::topic_callback(const std_msgs::msg::String & msg)
{
    // RCLCPP_INFO(get_logger(), "I heard: '%s'", msg.data.c_str()); // disable print to screen (to safe battery?)
    write(msg.data.c_str());
}

void LedSerialNode::on_configure(){
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

    // TODO: uncomment after testing
    if (comms_.connected())
    {
        comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(get_logger(), "Successfully configured!");
}

void LedSerialNode::write(std::string val){
    // RCLCPP_INFO(rclcpp::get_logger("LedSerialNode"), "Writing ...please wait...");
    comms_.set_value(val);
    // RCLCPP_INFO(get_logger(), "Successfully writen!"); // disable print to screen (to safe battery?)
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedSerialNode>());
  rclcpp::shutdown();
  return 0;
}
