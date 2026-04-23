from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    imu_node = Node(
        package="imu_pkg",
        executable="imu_node",
        name="imu_node",
        output="screen",
    )

    uva_node = Node(
        package="uva_pkg",
        executable="uva_node",
        name="uva_node",
        output="screen",
    )

    start_uva_after_imu = RegisterEventHandler(
        OnProcessStart(
            target_action=imu_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[uva_node],
                )
            ],
        )
    )

    return LaunchDescription([
        imu_node,
        start_uva_after_imu,
    ])
