# launch/planner_client.launch.py
import os, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xacro = FindExecutable(name="xacro")

    urdf = Command([xacro, " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " ur_type:=ur5e name:=ur"])
    srdf = Command([xacro, " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " name:=ur prefix:=\"\""])

    kin_path = os.path.join(get_package_share_directory("ur_moveit_config"),
                            "config", "kinematics.yaml")
    with open(kin_path) as f:
        kinematics = yaml.safe_load(f)

    return LaunchDescription([Node(
        package="ur_planner_client", executable="planner_client",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(urdf, value_type=str)},
            {"robot_description_semantic": ParameterValue(srdf, value_type=str)},
            {"robot_description_kinematics": kinematics},
            {"target_x": -0.707, "target_y": 0.001, "target_z": 0.001, "target_yaw": 0.707},
            {"low_x": 0.1, "low_y": 0.1, "low_z": -0.4, "low_yaw": 0.7},
            {"home_x": -0.5, "home_y": 0.5, "home_z": 0.5, "home_yaw": 0.5},
        ])])
