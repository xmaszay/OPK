from launch import LaunchDescription
from launch_ros.actions import Node


MAP_PATH = '/home/peter/Desktop/OPK/opk_ws/src/zadanie1/resources/opk-map.png'
MAP_RESOLUTION = 0.02

STATION_X = 21.0
STATION_Y = 7.5
STATION_RADIUS = 0.8


def generate_launch_description():
    map_node = Node(
        package='zadanie2_nodes',
        executable='map_node',
        name='map_node',
        output='screen',
        parameters=[
            {
                'map_path': MAP_PATH,
                'map_resolution': MAP_RESOLUTION
            }
        ]
    )

    player1_robot = Node(
        package='zadanie2_nodes',
        executable='robot_node',
        name='robot_node',
        namespace='player1',
        output='screen',
        parameters=[
            {
                'map_path': MAP_PATH,
                'map_resolution': MAP_RESOLUTION,
                'initial_x': 20.0,
                'initial_y': 8.0,
                'initial_theta': 0.0,
                'ghost_mode': False,
                'linear_acceleration': 3.0,
                'angular_acceleration': 2.0,
                'linear_emergency_deceleration': 2.0,
                'angular_emergency_deceleration': 2.0,
                'command_duration': 0.5,
                'simulation_period_ms': 20,
                'publish_period_ms': 50
            }
        ]
    )

    player1_lidar = Node(
        package='zadanie2_nodes',
        executable='lidar_node',
        name='lidar_node',
        namespace='player1',
        output='screen',
        parameters=[
            {
                'map_path': MAP_PATH,
                'map_resolution': MAP_RESOLUTION,
                'base_frame_id': 'player1/base_link',
                'max_range': 8.0,
                'beam_count': 360,
                'first_ray_angle': -3.14159,
                'last_ray_angle': 3.14159,
                'publish_period_ms': 100
            }
        ]
    )

    player2_robot = Node(
        package='zadanie2_nodes',
        executable='robot_node',
        name='robot_node',
        namespace='player2',
        output='screen',
        parameters=[
            {
                'map_path': MAP_PATH,
                'map_resolution': MAP_RESOLUTION,
                'initial_x': 23.0,
                'initial_y': 8.0,
                'initial_theta': 3.14,
                'ghost_mode': True,
                'linear_acceleration': 3.0,
                'angular_acceleration': 2.0,
                'linear_emergency_deceleration': 2.0,
                'angular_emergency_deceleration': 2.0,
                'command_duration': 0.5,
                'simulation_period_ms': 20,
                'publish_period_ms': 50
            }
        ]
    )

    player2_lidar = Node(
        package='zadanie2_nodes',
        executable='lidar_node',
        name='lidar_node',
        namespace='player2',
        output='screen',
        parameters=[
            {
                'map_path': MAP_PATH,
                'map_resolution': MAP_RESOLUTION,
                'base_frame_id': 'player2/base_link',
                'max_range': 8.0,
                'beam_count': 360,
                'first_ray_angle': -3.14159,
                'last_ray_angle': 3.14159,
                'publish_period_ms': 100
            }
        ]
    )

    game_node = Node(
        package='zadanie2_nodes',
        executable='game_node',
        name='game_node',
        output='screen',
        parameters=[
            {
                'map_path': MAP_PATH,
                'map_resolution': MAP_RESOLUTION,
                'max_capacity': 3,
                'trash_count': 12,
                'trash_radius': 0.25,
                'collect_distance': 0.7,
                'station_x': STATION_X,
                'station_y': STATION_Y,
                'station_radius': STATION_RADIUS
            }
        ]
    )

    bot_node = Node(
        package='zadanie2_nodes',
        executable='bot_node',
        name='bot_node',
        output='screen',
        parameters=[
            {
                'max_capacity': 3,
                'station_x': STATION_X,
                'station_y': STATION_Y,
                'target_distance': 0.8,
                'linear_speed': 1.5,
                'angular_gain': 1.5,
                'angle_tolerance': 0.25
            }
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        map_node,
        player1_robot,
        player1_lidar,
        player2_robot,
        player2_lidar,
        game_node,
        bot_node,
        rviz
    ])