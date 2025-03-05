from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    

    serial = Node(
        package="xenopkg",
        executable="serialpublish",
        output="screen"
    )

    lqr = Node(
        package="xenopkg",
        executable="xenolqrcontroller",
        output="screen"
    )
    # task = Node(
    #     package="xenopkg",
    #     executable="mainpendulum",
    #     output="screen",
    #     # parameters=[{
    #     #        'soglia_':0.5,
    #     #    }]
    # )

    return LaunchDescription([
        serial,
        lqr
        # task
        ])