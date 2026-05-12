from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('zadanie2_nodes')
    config_file = os.path.join(pkg_share, 'config', 'duel.yaml')

    map_node = Node(
        package='zadanie2_nodes',
        executable='map_node',
        name='map_node',
        output='screen',
        parameters=[config_file]
    )

    player1_robot = Node(
        package='zadanie2_nodes',
        executable='robot_node',
        name='robot_node',
        namespace='player1',
        output='screen',
        parameters=[config_file]
    )

    player1_lidar = Node(
        package='zadanie2_nodes',
        executable='lidar_node',
        name='lidar_node',
        namespace='player1',
        output='screen',
        parameters=[config_file]
    )

    player2_robot = Node(
        package='zadanie2_nodes',
        executable='robot_node',
        name='robot_node',
        namespace='player2',
        output='screen',
        parameters=[config_file]
    )

    player2_lidar = Node(
        package='zadanie2_nodes',
        executable='lidar_node',
        name='lidar_node',
        namespace='player2',
        output='screen',
        parameters=[config_file]
    )

    game_node = Node(
        package='zadanie2_nodes',
        executable='game_node',
        name='game_node',
        output='screen',
        parameters=[config_file]
    )

    bot_node = Node(
        package='zadanie2_nodes',
        executable='bot_node',
        name='bot_node',
        output='screen',
        parameters=[config_file]
    )

    visualization_node = Node(
        package='zadanie2_nodes',
        executable='visualization_node',
        name='visualization_node',
        output='screen'
    )

    web_node = Node(
        package='zadanie2_nodes',
        executable='web_node',
        name='web_node',
        output='screen',
        parameters=[config_file]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/peter/.rviz2/OPK_final_zad.rviz']
    )

    return LaunchDescription([
        map_node,
        player1_robot,
        player1_lidar,
        player2_robot,
        player2_lidar,
        game_node,
        bot_node,
        visualization_node,
        web_node,
        rviz
    ])