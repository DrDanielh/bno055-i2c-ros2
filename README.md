Node for BNO055 IMU for ROS2. Modified following ROS (1) node to work in ROS2:
https://github.com/dheera/ros-imu-bno055


Add following lines to your launch file:

        bno055_node = Node(package='bno055_i2c_ros2',
                            namespace="",
                            executable='bno055_i2c_ros2')

This will publish data on following topics:

        /bno055/data
        /bno055/raw
        /bno055/mag             
        /bno055/temp
        /bno055/status


Frame ID on IMU messages default to "bno055", but can be parsed as a parameter in launch file.
