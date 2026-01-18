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

def local(tag: str) -> str:
    # Strip namespace: "{ns}tag" -> "tag"
    return tag.split('}', 1)[1] if '}' in tag else tag

def print_xml(node: ET.Element, indent: int = 0):
    pad = "  " * indent
    attrs = " ".join(f'{k}="{v}"' for k, v in node.attrib.items())
    open_tag = f"<{local(node.tag)}{(' ' + attrs) if attrs else ''}>"
    text = (node.text or "").strip()

    if text:
        print(f"{pad}{open_tag} {text}")
    else:
        print(f"{pad}{open_tag}")

    for child in list(node):
        print_xml(child, indent + 1)

    print(f"{pad}</{local(node.tag)}>")

def modify_world_file(pkg_path, world_file, config_file):
    world_file_updated = "/tmp/temp.sdf"
    world_tree = ET.parse(world_file)
    world_root = world_tree.getroot()
    if config_file.get("environment_setup") is not None:
        for key in config_file["environment_setup"].keys():
            model_sdf_path = os.path.join(pkg_path,"description" ,"worlds" ,"component" ,config_file["environment_setup"][key]["model_file"])
            model_tree = ET.parse(model_sdf_path)
            model_root = model_tree.getroot()
            pose=config_file["environment_setup"][key]["pose"]
            # print_xml(model_root)
            model_root.find('pose').text = pose
            for uri in model_root.iter("uri"):
                if uri.text:  # make sure it has text
                    uri.text = os.path.join(pkg_path, uri.text)
            world_root.find('world').append(model_root)

    world_root.find('.//latitude_deg').text = str(config_file["world_setup"]["latitude_origin"])
    world_root.find('.//longitude_deg').text = str(config_file["world_setup"]["longitude_origin"])
    ET.indent(world_tree, space="  ", level=0)
    # Save merged SDF
    world_tree.write(world_file_updated, encoding='utf-8', xml_declaration=True)
    return world_file_updated

def generate_launch_description():
    use_sim_time_launch_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true')

    pkg_path = os.path.join(get_package_share_directory("multibot_model"))
    world_file = os.path.join(pkg_path,'worlds','world.sdf')

    with open(os.path.join(pkg_path, 'config', 'sim_setup.yaml'), 'r') as file:
        config_file = yaml.safe_load(file)
    world_file_updated = modify_world_file(pkg_path, world_file, config_file)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        # launch_arguments={'gz_args': world_file_updated + ' -r --render-engine ogre', 'on_exit_shutdown': 'true'}.items(),
        launch_arguments={'gz_args': world_file_updated + ' -r -v 4 --render-engine ogre', 'on_exit_shutdown': 'true'}.items(),
    )

    # Launch!
    return LaunchDescription([
        gazebo,
        use_sim_time_launch_arg
    ])