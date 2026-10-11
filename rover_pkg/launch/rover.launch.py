from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='rover_pkg',
            executable='motor_driver_node',
            name='motor_driver_node',
            output='screen'
        ),

        Node(
            package='rover_pkg',
            executable='ultrasonic_node',
            name='ultrasonic_node',
            output='screen'
        ),

        Node(
            package='rover_pkg',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        Node(
            package='rover_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        Node(
            package='rover_pkg',
            executable='odometry_node',
            name='odometry_node',
            output='screen'
        ),

        Node(
            package='rover_pkg',
            executable='safety_node',
            name='safety_node',
            output='screen'
        ),

	    Node(
        	    package='rover_pkg',
        	    executable='mission_node',
        	    name='mission_node',
        	    output='screen'
	    ),

	    Node(
        	    package='rover_pkg',
        	    executable='depth_node',
        	    name='depth_node',
        	    output='screen'
	    ),

	    Node(
        	    package='rover_pkg',
        	    executable='camera_info_node',
        	    name='camera_info_node',
        	    output='screen'
	    ),

	    Node(
        		package='rtabmap_slam',
        		executable='rtabmap',
        		name='rtabmap',
        		output='screen',
                parameters=[{
                    'frame_id': 'base_link',
                    'subscribe_depth': True,
                    'subscribe_rgb': True,
                    'subscribe_stereo': False,
                    'subscribe_rgbd': False,
                    'subscribe_odom_info': False,
                    'approx_sync': True,
                    'sync_queue_size': 30,
                    'topic_queue_size': 30,
                    'Rtabmap/DetectionRate': '1',
                    'RGBD/NeighborLinkRefining': 'true',
                    'Grid/FromDepth': 'true',
                    'Reg/Strategy': '0',
                    'Vis/EstimationType': '0',
                    'Rtabmap/CreateIntermediateNodes': 'true',
                    'cloud_output_voxelized': True,
                    'latch': False,
		    'Vis/MinInliers': '10',
		    'Rtabmap/LoopThr': '0.11',
                }],
        		remappings=[
            	    ('rgb/image', '/camera/image_raw'),
            	    ('depth/image', '/camera/depth'),
            	    ('rgb/camera_info', '/camera/camera_info'),
            	    ('odom', '/odom'),
        		],
        		arguments=['-d', '--ros-args', '--log-level', 'rtabmap:=warn']
	    ),

        # Static transforms
        Node(
       	    package='tf2_ros',
        	    executable='static_transform_publisher',
        	    name='imu_tf',
        	    arguments=[
            	'--x', '0', '--y', '0', '--z', '0',
            	'--roll', '0', '--pitch', '0', '--yaw', '0',
           		'--frame-id', 'base_link',
            	'--child-frame-id', 'imu_sensor'
        		]
	    ),

	    Node(
        	    package='tf2_ros',
                executable='static_transform_publisher',
        	    name='ultrasonic_tf',
        	    arguments=[
            	'--x', '0.1', '--y', '0', '--z', '0.05',
            	'--roll', '0', '--pitch', '0', '--yaw', '0',
            	'--frame-id', 'base_link',
            	'--child-frame-id', 'ultrasonic_sensor'
        		]
	    ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=[
                '--x', '0.1', '--y', '0', '--z', '0.20',
                '--roll', '0', '--pitch', '0.1745', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'camera',
                '--child-frame-id', 'camera_optical'
            ]
        ),
        
        # Nav2
        # Nav2 — only the nodes we actually need
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('rover_pkg'),
                'config', 'nav2_params.yaml'
            )]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('rover_pkg'),
                'config', 'nav2_params.yaml'
            )]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('rover_pkg'),
                'config', 'nav2_params.yaml'
            )]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('rover_pkg'),
                'config', 'nav2_params.yaml'
            )]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }]
        ),

    ])
