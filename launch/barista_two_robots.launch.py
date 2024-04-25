import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    pkg_description_name = "barista_robot_description"
    share_dir = get_package_share_directory(pkg_description_name)
    install_dir = get_package_prefix(pkg_description_name)

    robot_name_rick = 'rick'
    robot_name_morty = 'morty'

    # This is to find the models inside the models folder in package_name
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))
    # robot_model_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(share_dir, 'xacro', 'barista_robot_model.urdf.xacro')

    # convert XACRO file into URDF
    doc_robot_rick = xacro.process_file(xacro_file, mappings={"r_n": robot_name_rick})
    doc_robot_morty = xacro.process_file(xacro_file, mappings={"r_n": robot_name_morty})    
    params_robot_rick = {'use_sim_time': True, 'robot_description': doc_robot_rick.toxml(), 'frame_prefix': '/' + robot_name_rick + '/'}
    params_robot_morty = {'use_sim_time': True, 'robot_description': doc_robot_morty.toxml(), 'frame_prefix': '/' + robot_name_morty + '/'}


    # Robot State Publisher
    robot_state_publisher_node_rick = Node(
        package='robot_state_publisher',
        namespace=robot_name_rick,
        executable='robot_state_publisher',
        output='screen',
        parameters=[params_robot_rick]
    )

    robot_state_publisher_node_morty = Node(
        package='robot_state_publisher',
        namespace=robot_name_morty,
        executable='robot_state_publisher',
        output='screen',
        parameters=[params_robot_morty]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={"verbose": "false", 'pause': 'false'}.items()
    )

    spawn_entity_rick = Node(package='gazebo_ros', 
                    namespace=robot_name_rick,
                    executable='spawn_entity.py',
                    arguments=['-entity', robot_name_rick, '-x', '1.0', '-y', '1.0', '-z', '0.2',
                                '-topic', '/' + robot_name_rick + '/' + 'robot_description'],
                    output='screen')

    spawn_entity_morty = Node(package='gazebo_ros', 
                    namespace=robot_name_morty,
                    executable='spawn_entity.py',
                    arguments=['-entity', robot_name_morty, '-x', '2.0', '-y', '1.0', '-z', '0.2',
                                '-topic', '/' + robot_name_morty + '/' + 'robot_description'],
                    output='screen')

    static_transform_publisher_node_rick = Node(package='tf2_ros', 
                    executable='static_transform_publisher',
                    arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', '/world', '--child-frame-id', '/' + robot_name_rick + '/odom'],
                    output='screen')

    static_transform_publisher_node_morty = Node(package='tf2_ros', 
                    executable='static_transform_publisher',
                    arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                    '--frame-id', '/world', '--child-frame-id', '/' + robot_name_morty + '/odom'],
                    output='screen')
                    
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(pkg_description_name), 'rviz', 'barista_two_bot.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription(
        [    
            robot_state_publisher_node_rick,
            robot_state_publisher_node_morty,
            gazebo,
            spawn_entity_rick,
            spawn_entity_morty,
            static_transform_publisher_node_rick,
            static_transform_publisher_node_morty,
            rviz_node
        ]
    )