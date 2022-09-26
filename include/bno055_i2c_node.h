#ifndef BNO55_I2C_ROS2_INCLUDE_BNO055_I2C_NODE_H_
#define BNO55_I2C_ROS2_INCLUDE_BNO055_I2C_NODE_H_

#include <imu_bno055/bno055_i2c_driver.h>
#include "watchdog/watchdog.h"
#include <csignal>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
//#include "std_srvs/msg/Trigger.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

class BNO055I2CNode : public rclcpp::Node {
    public:
        BNO055I2CNode();

        //bool onSrvReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    private:
        void update();
        bool readAndPublish();

        std::unique_ptr<imu_bno055::BNO055I2CDriver> imu;

        std::string node_name = "bno055";
        std::string param_device;
        int param_address;
        double param_rate;
        std::string param_frame_id;

        diagnostic_msgs::msg::DiagnosticStatus current_status;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_data;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag;
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status;

        // ros::Publisher pub_data;
        // ros::Publisher pub_raw;
        // ros::Publisher pub_mag;
        // ros::Publisher pub_temp;
        // ros::Publisher pub_status;
        // ros::ServiceServer srv_reset;

        //std::unique_ptr<ros::Rate> rate;
        rclcpp::TimerBase::SharedPtr update_timer;

        watchdog::Watchdog watchdog;

        int seq;
};


#endif // BNO55_I2C_ROS2_INCLUDE_BNO055_I2C_NODE_H_
