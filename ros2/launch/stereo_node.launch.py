import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  ld = LaunchDescription()
  stereo_visual_odometry_node = Node(
    package='stereo_visual_odometry',
    executable='stereo_visual_odometry_node',
    output='screen',
    parameters=[
        {'topicname_image_left': '/camera/infra1/image_rect_raw'},
        {'topicname_image_right': '/camera/infra2/image_rect_raw'},
        {'topicname_pose': '/stereo_visual_odometry/pose'},
        {'topicname_trajectory': '/stereo_visual_odometry/trajectory'},
        {'topicname_map_points': '/stereo_visual_odometry/map_points'},
        {'topicname_debug_image': '/stereo_visual_odometry/debug/image'},
        {'directory_parameter': '/home/kch/ros2_ws/src/stereo_visual_odometry/config/d435/'}
    ]
    # parameters = [config]
  )

  ld.add_action(stereo_visual_odometry_node)
  return ld