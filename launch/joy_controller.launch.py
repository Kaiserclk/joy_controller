from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,RegisterEventHandler)
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
def generate_launch_description():

    joy_node = Node(
        package='joy',
        executable='joy_node',
    )
    joy_controller_node = LifecycleNode(
          package='joy_controller',
          executable='joy_diff',
          name='joy_diff',
          output='screen',
          namespace=''
    )
    #发送事件
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(joy_controller_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        ),
    )
    #注册监听事件，当joy_controller_node配置完成进入inactive时触发
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=joy_controller_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(joy_controller_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
    )
    return LaunchDescription([
        joy_node,
        joy_controller_node,
        configure_event,
        activate_event
    ])