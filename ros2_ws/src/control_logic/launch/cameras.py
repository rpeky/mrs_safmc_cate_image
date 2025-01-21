from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            namespace='camera',
            executable='camera_node',
            name='cam0',
            arguments=[
                '--ros-args',
                '-p', 'camera:=0',
                '-p', 'width:=1920',
                '-p', 'height:=1080',
                '-p', 'format:=YUYV']

        ),
        Node(
            package='camera_ros',
            namespace='camera',
            executable='camera_node',
            name='cam1',
            arguments=[
                '--ros-args',
                '-p', 'camera:=1',
                '-p', 'width:=1920',
                '-p', 'height:=1080',
                '-p', 'format:=YUYV']
        ),
        Node(
            package='safmc_2024',
            namespace='image_converter',
            executable='image_converter',
            name='cam0',
            parameters=[
                {"display": False}
            ],
            remappings=[
                ( "/image_raw", "/camera/cam0/image_raw" ),
                ( "/coordinates", "/image_converter/cam0/coordinates" )
            ]
        )
    ])

front_image_converter = Node(
                    package='safmc_2024',
                    namespace='image_converter',
                    executable='image_converter',
                    name='cam1',
                    parameters=[
                        {"display": False}
                    ],
                    remappings=[
                        ( "/image_raw", "/camera/cam1/image_raw" ),
                        ( "/coordinates", "/image_converter/cam1/coordinates" )
                    ]
                )
