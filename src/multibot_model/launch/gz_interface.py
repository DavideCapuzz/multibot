import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge

import xacro
import subprocess
import xml.etree.ElementTree as ET
import yaml

def generate_launch_description():
    model_name= os.getenv('ROBOT_NAME')
    pos_x= os.getenv('STARTPOS_X')
    pos_y= os.getenv('STARTPOS_Y')
    pos_z= os.getenv('STARTPOS_Z')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', model_name,
                    '-x', pos_x,
                    '-z', pos_y,
                    '-Y', pos_z,
                    '-topic', '/robot_description'],
                 output='screen')
    
    # custom bridge node designed by me
    # gz_interface = Node(
    #         package='gz_interface',
    #         namespace='gz_node',
    #         executable='gz_node',
    #         name='gz_node'
    #     )

    # Bridge to forward tf and joint states to ros2
    topic_nav = "/world/base_world/model/bot1/link/base_footprint/sensor/navsat/navsat"
    topic_imu = "/world/base_world/model/bot1/link/base_footprint/sensor/imu_sensor/imu"

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/bot1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/bot1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/bot1/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/base_world/model/bot1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'{topic_imu}@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'{topic_nav}@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/scangz@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/bot1/odometry', '/odom'),
            ('/model/bot1/tf', '/tf'),
            ('/world/base_world/model/bot1/joint_state', '/joint_states'),
            (topic_nav, '/navsat'),
            (topic_imu, '/imu'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    static_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        # arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'wheele']
    )


    # Launch!
    return LaunchDescription([
        bridge,
        spawn
    ])