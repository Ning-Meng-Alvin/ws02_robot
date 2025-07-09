from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="KR210_R2700_extra", package_name="kuka_kr210_moveit_config")
        .to_moveit_configs()
    )

    tf_init_node = Node(
        package="kuka_tf_initializer",
        executable="kuka_tf_initializer",
        name="kuka_tf_initializer_node",
        output="screen"
    )

    demo_launch = generate_demo_launch(moveit_config)

    tf_republish_node = Node(
        package="robot_tf_broadcaster", 
        executable="robot_tf_broadcaster",  
        name="robot_tf_broadcaster_node",
        output="screen"
    )

    collision_node = Node(
        package="collision_object_manager_cpp",  
        executable="collision_object_node",
        name="collision_object_node",
        output="screen"
    )

    cartesian_action_node = Node(
        package="kuka_communication_server",
        executable="cartesian_follow_action_server",
        name="cartesian_follow_action_server_node",
        output="screen"
    )

    pose_check_action_node = Node(
        package="kuka_communication_server",
        executable="pose_check_action_server",
        name="pose_check_action_server_node",
        output="screen"
    )

    return LaunchDescription([
        tf_init_node,
        TimerAction(
            period=2.0,
            actions=[
                *demo_launch.entities,
                TimerAction(
                    period=2.0,
                    actions=[
                        tf_republish_node,
                        TimerAction(
                            period=2.0,
                            actions=[
                                collision_node,
                                cartesian_action_node,
                                pose_check_action_node]
                        )
                    ]
                )
            ]
        )
    ])