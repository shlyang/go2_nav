import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

print("hei hei hei")

def generate_launch_description():
    # root_path = '/unitree/module/graph_pid_ws'
    cur_path = '/home/unitree/navi_ws/src/unitree_nav2'
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pc2scan_dir = get_package_share_directory('pointcloud_to_laserscan')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # map_yaml_path = LaunchConfiguration('map',default=os.path.join(os.path.abspath('.'), 'src','task','maps','map','default.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(cur_path,'config','my_robot.yaml'))
    slam_toolbox_param_path = LaunchConfiguration('slam_params_file',default=os.path.join(cur_path,'config','my_slam_toolbox.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gazebo) clock if true'),
        # DeclareLaunchArgument('map',default_value=map_yaml_path,description='Full path to map file to load'),
        DeclareLaunchArgument('params_file',default_value=nav2_param_path,description='Full path to [nav2] param file to load'),
        DeclareLaunchArgument('slam_params_file',default_value=slam_toolbox_param_path,description='Full path to [slam_toolbox] param file to load'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('hesai_lidar'), 'launch'),
                                           '/hesai_lidar_launch.py'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pc2scan_dir,'/launch','/my_pointcloud_to_laserscan_launch.py'])
        ),
        
        IncludeLaunchDescription(
            # PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/navigation_launch.py']),
            launch_arguments={
                # 'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_toolbox_dir,'/launch','/online_async_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': slam_toolbox_param_path}.items(),
        ),
        
        Node(
            package="dog_control",
            executable="dog_control",
            output='screen'
        ),
        
        #Node(
        #    package='rslidar_sdk',
        #    executable='rslidar_sdk_node',
        #    name='rslidar_sdk_node',
        #    output='screen'
        #),

        # TimerAction(period=2.0,actions= [
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([os.path.join(
        #             get_package_share_directory('lio_sam_ros2'), 'launch'),
        #             '/relio.launch.py'])
        #     )
        # ]),
        
        # Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', rviz_config_dir],
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    output='screen'
        # ),

    ])