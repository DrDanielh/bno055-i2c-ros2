Add following lines to your launch file:

        bno055_node = Node(package='bno055',
                            namespace="",
                            executable='bno055_i2c')

This will publish data on following topics:

        /bno055/data            # sensor_msgs/Imu Message
        /bno055/raw
        /bno055/mag
        /bno055/temp
        /bno055/status
