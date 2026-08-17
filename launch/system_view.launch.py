"""Launch the operator view.

Runs on ONE machine and watches every tower. Which machine does not matter:
ROS 2 has no master, so this is a gateway for the browser (which cannot speak
DDS), not a coordinator. The towers never learn it exists, and killing it
affects nothing but the page.

Usage:
    ros2 launch two_towers system_view.launch.py

    # with the MJPEG debug streams embedded (costs each tower ~26% of its
    # detection loop, so it is off by default)
    ros2 launch two_towers system_view.launch.py \\
        stream_a:=http://orthanc.local:5000/video_feed \\
        stream_b:=http://baraddur.local:5001/video_feed

Deliberately NOT part of tower.launch.py: a tower should come up and track
whether or not anyone is watching, and the view should be restartable without
touching a turret.
"""

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'port', default_value='8080',
            description='HTTP port for the operator view.',
        ),
        # Empty by default. Streaming is a debug tool, not a feature: annotation
        # and JPEG encoding run inside the detection callback and cost ~44 ms of
        # a ~123 ms loop.
        DeclareLaunchArgument(
            'stream_a', default_value='',
            description='MJPEG URL for the first tower, or empty for no video.',
        ),
        DeclareLaunchArgument(
            'stream_b', default_value='',
            description='MJPEG URL for the second tower, or empty for no video.',
        ),
    ]

    # tower_ids and tower_labels are NOT launch arguments. Passing a list
    # through a LaunchConfiguration means handing launch a string and hoping it
    # coerces to a string array, which is exactly the kind of thing that works
    # on a laptop and fails on a robot. The node's defaults are already the two
    # towers; a third tower is `ros2 run ... -p tower_ids:="[a,b,c]"` or an
    # edit here, both of which are explicit.

    view = Node(
        package='two_towers',
        executable='system_view_node.py',
        name='system_view',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            # The ParameterValue wraps the WHOLE list, not each element.
            # Wrapping elements individually raises "Expected 'subvalue' to be
            # one of [...]" at load time, and a bare list of substitutions is
            # ambiguous -- launch would concatenate them into one string rather
            # than build a two-element array. list[str] states which is meant.
            'stream_urls': ParameterValue(
                [LaunchConfiguration('stream_a'), LaunchConfiguration('stream_b')],
                value_type=list[str],
            ),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription(args + [view])
