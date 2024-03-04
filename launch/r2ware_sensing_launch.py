import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    # Include astro_pro_plus.launch.xml
    astro_pro_plus_launch_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astra_camera"),
                "launch/astro_pro_plus.launch.xml",
            )
        )
    )

    return LaunchDescription([
        # Include Ackman_driver_R2 node
        Node(
            package='r2ware_sensing',
            executable='r2ware_sensing_node',
            output='screen',
            prefix=[],
        ),
        Node(
            package='yahboomcar_bringup',
            executable='Ackman_driver_R2',
            output='screen',
            prefix=[],
        ),
        # Include astro_pro_plus.launch.xml
        astro_pro_plus_launch_include
    ])
