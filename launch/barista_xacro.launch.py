import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    pkg_description_name = "barista_robot_description"
    share_dir = get_package_share_directory(pkg_description_name)
    install_dir = get_package_prefix(pkg_description_name)

    robot_name = 'barista_bot'

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

    # convert XACRO file into URDF
    xacro_file = os.path.join(share_dir, 'xacro', 'barista_robot_model.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={ "robot_name": robot_name})

    params = {'use_sim_time': True, 'robot_description': doc.toxml(), 'mappings': {"robot_name": robot_name}, 'frame_prefix': '/' + robot_name + '/'}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        namespace=robot_name,
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={"verbose": "false", 'pause': 'false'}.items(),
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-entity', 'barista_bot', '-x', '1.0', '-y', '1.0', '-z', '0.2',
                                '-topic', '/' + robot_name + '/' + 'robot_description'],
                    output='screen')
                    
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(pkg_description_name), 'rviz', 'barista_bot.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription(
        [    
            robot_state_publisher_node,        
            gazebo,
            spawn_entity,
            rviz_node
        ]
    )