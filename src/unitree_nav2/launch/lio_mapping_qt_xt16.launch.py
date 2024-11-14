import os
import sys
from readline import get_history_item
from sysconfig import get_path, get_path_names, get_paths

import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

unitree_slam_interfaces_path = os.path.join(os.path.abspath('.'), 'config_files', 'QT_Server_config')

def my_get_path():
    # # 参数是 map_name:=test
    # my_map_name = sys.argv[4]
    # mapname = my_map_name[10:]
    # return os.path.join(os.path.abspath('.'),'src','task','maps','pcd',mapname)
    argc = len(sys.argv)

    if argc < 5:
        mapname = 'default'+ '/'

    else:
        # 参数是 map_name:=test
        my_map_name = sys.argv[4]
        mapname = my_map_name[10:] + '/'

    return os.path.join(os.path.abspath('.'),'config_files','lio_sam_config','maps','pcd',mapname)



def generate_launch_description():
 
    map_name_dir = LaunchConfiguration(
        'map_name_dir',
        default=os.path.join(
            my_get_path()
            )
    )

    mapping_param_dir = launch.substitutions.LaunchConfiguration(
        'mapping_param_dir',
        default=os.path.join(
            os.path.abspath('.'),
            'config_files',
            'lio_sam_config',
            'params_xt16.yaml'))
    
    unitree_slam_interfaces_param = launch.substitutions.LaunchConfiguration(
        'unitree_slam_interfaces_config_dir',
        default=os.path.join(
            unitree_slam_interfaces_path,  
            'configs',
            'params.yaml'))
    
    rs_lidar_cheak = launch_ros.actions.Node(
        package="QT_Server",
        executable="rs_lidar_cheak",
        parameters=[{unitree_slam_interfaces_param}],
        output='screen'
        )
    
    rviz_config = os.path.join(os.path.abspath('.'), 'src', 'task', 'rviz2', 'bulid_map.rviz')

    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default= 'True' )

    dogodom = launch_ros.actions.Node(
        package='lio_sam_ros2',
        executable='lio_sam_ros2_dogOdomForMapping',
        parameters=[{mapping_param_dir},{'use_sim_time':use_sim_time}],
        output='screen'
        )

    imu_pre = launch_ros.actions.Node(
        package='lio_sam_ros2',
        executable='lio_sam_ros2_imuPreintegration',
        parameters=[{mapping_param_dir},{'use_sim_time': use_sim_time}],
        output='screen'
        )

    imagep = launch_ros.actions.Node(
        package='lio_sam_ros2',
        executable='lio_sam_ros2_imageProjection',
        parameters=[{mapping_param_dir},{'use_sim_time': use_sim_time}],
        output='screen'
        )

    featurex = launch_ros.actions.Node(
        package='lio_sam_ros2',
        executable='lio_sam_ros2_featureExtraction',
        parameters=[{mapping_param_dir},{'use_sim_time': use_sim_time}],
        output='screen'
        )

    map_opt = launch_ros.actions.Node(
        package='lio_sam_ros2',
        executable='lio_sam_ros2_mapOptmization',
        parameters=[{mapping_param_dir},{unitree_slam_interfaces_param},{'savePCDDirectory': map_name_dir},{'use_sim_time': use_sim_time}],
        output='screen'
        )

    # tf_static = launch_ros.actions.Node(#lz:该静态变换改在程序中
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'rslidar']
    #     )

    logrecord= launch_ros.actions.Node(
        package='lio_sam_ros2',
        executable='lio_sam_ros2_logRecord',
        parameters=[{mapping_param_dir},{'use_sim_time': use_sim_time}],
        output='screen'
        )
    # occ_grid_map = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('occ_grid_mapping'), 'launch'),
    #      '/mapping.launch.py'])
    #   )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'mapping_param_dir',
            default_value=mapping_param_dir,
            description='Full path to mapping parameter file to load'),
        rs_lidar_cheak,
        dogodom,
        imu_pre,
        imagep,
        # featurex,   
        map_opt,
        # logrecord,
        # occ_grid_map,
        #rviz_node,
            ])
