#include "bno055_i2c_node.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <chrono>

BNO055I2CNode::BNO055I2CNode() : Node("bno055_i2c_ros2") {
    RCLCPP_INFO(this->get_logger(), "Instantiated bno055_i2c_ros2 node.");
    
    this->declare_parameter<std::string>("device", "/dev/i2c-1");
    this->declare_parameter<int>("address", BNO055_ADDRESS_A);
    this->declare_parameter<std::string>("frame_id", "bno055");
    this->declare_parameter<double>("rate", 100);

    param_device = this->get_parameter("device").as_string();
    param_address = this->get_parameter("address").as_int();
    param_frame_id = this->get_parameter("frame_id").as_string();;
    param_rate = this->get_parameter("rate").as_double();
 
    imu = std::make_unique<imu_bno055::BNO055I2CDriver>(param_device, param_address);

    imu->init();

    pub_data = this->create_publisher<sensor_msgs::msg::Imu>(topic_namespace+"/data", 1);
    pub_raw = this->create_publisher<sensor_msgs::msg::Imu>(topic_namespace+"/raw", 1);
    pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>(topic_namespace+"/mag", 1);
    pub_temp = this->create_publisher<sensor_msgs::msg::Temperature>(topic_namespace+"/temp", 1);
    pub_status = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(topic_namespace+"/status", 1);

    seq = 0;


    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::msg::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::msg::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::msg::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::msg::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::msg::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::msg::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);

    update_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/param_rate)),
                                          std::bind(&BNO055I2CNode::update, this));

}

void BNO055I2CNode::update() {
    if(readAndPublish()) {
        watchdog.refresh();
    }
}

bool BNO055I2CNode::readAndPublish() {
    imu_bno055::IMURecord record;

    try {
        record = imu->read();
    } catch(const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }

    auto time = this->get_clock()->now();

    sensor_msgs::msg::Imu msg_raw;
    msg_raw.header.stamp = time;
    msg_raw.header.frame_id = param_frame_id;
    msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::msg::MagneticField msg_mag;
    msg_mag.header.stamp = time;
    msg_mag.header.frame_id = param_frame_id;
    msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
    msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
    msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;
    msg_mag.magnetic_field_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    sensor_msgs::msg::Imu msg_data;
    msg_data.header.stamp = time;
    msg_data.header.frame_id = param_frame_id;

    double fused_orientation_norm = std::pow(
      std::pow(record.fused_orientation_w, 2) +
      std::pow(record.fused_orientation_x, 2) +
      std::pow(record.fused_orientation_y, 2) +
      std::pow(record.fused_orientation_z, 2), 0.5);

    msg_data.orientation.w = (double)record.fused_orientation_w / fused_orientation_norm;
    msg_data.orientation.x = (double)record.fused_orientation_x / fused_orientation_norm;
    msg_data.orientation.y = (double)record.fused_orientation_y / fused_orientation_norm;
    msg_data.orientation.z = (double)record.fused_orientation_z / fused_orientation_norm;
    msg_data.orientation_covariance = {0.0159, 0.0, 0.0, 0.0, 0.0159, 0.0, 0.0, 0.0, 0.0159};
    msg_data.linear_acceleration.x = (double)record.fused_linear_acceleration_x / 100.0;
    msg_data.linear_acceleration.y = (double)record.fused_linear_acceleration_y / 100.0;
    msg_data.linear_acceleration.z = (double)record.fused_linear_acceleration_z / 100.0;
    msg_data.linear_acceleration_covariance = {0.017, 0.0, 0.0, 0.0, 0.017, 0.0, 0.0, 0.0, 0.017};
    msg_data.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_data.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_data.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;
    msg_data.angular_velocity_covariance = {0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04};
    
    sensor_msgs::msg::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;
    msg_temp.temperature = (double)record.temperature;

    pub_data->publish(msg_data);
    pub_raw->publish(msg_raw);
    pub_mag->publish(msg_mag);
    pub_temp->publish(msg_temp);

    if((seq++) % 50 == 0) {
        current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
        current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
        current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
        current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
        current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
        current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
        pub_status->publish(current_status);
    }

    return true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BNO055I2CNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

