import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters
    namespace = ''

    # Instantiate launch description
    ld = LaunchDescription()
   
    # Set up executables
    bno055_node = Node(package='bno055',
                        namespace=namespace,
                        executable='bno055_i2c')
    

    
    # Add nodes to launch description
    ld.add_action(bno055_node)

    return ld
