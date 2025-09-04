from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    namespace = "autoloader"

    coords_node = Node(
        package="autoloader_data",
        executable="coords_manager",
        namespace=namespace,
    )

    controller_node = Node(
        package="autoloader_controller",
        executable="controller",
        namespace=namespace,
    )

    runner_node = Node(
        package="autoloader_runner",
        executable="runner",
        namespace=namespace,
    )

    gui_node = Node(
        package="autoloader_gui",
        executable="gui",
        namespace=namespace,
    )

    shutdown_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gui_node,
            on_exit=EmitEvent(event=Shutdown()),
        )
    )

    return LaunchDescription(
        [
            coords_node,
            controller_node,
            runner_node,
            gui_node,
            shutdown_manager,
        ]
    )
