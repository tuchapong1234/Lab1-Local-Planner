from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    launch_description = LaunchDescription()

    purepursuit_cont = Node(
        package='lab1',
        executable='purepursuit.py'
    )

    pose2vec = Node(
        package='lab1',
        executable='pose2vector.py'
    )

    vff_cont = Node(
        package='lab1',
        executable='vff.py'
    )

    launch_description.add_action(vff_cont)
    launch_description.add_action(pose2vec)
    launch_description.add_action(purepursuit_cont)
    return launch_description