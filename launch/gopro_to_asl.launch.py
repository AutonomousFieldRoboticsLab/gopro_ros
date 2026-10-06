"""ROS 2 launch file. For ROS 1 use gopro_to_asl.launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name, default, description)
STRING_ARGS = [
    ("gopro_video", "/vid.mp4", "GoPro video file"),
    ("gopro_folder", "/vid_folder", "Folder with GoPro video chunks (used with multiple_files)"),
    ("asl_dir", "/path/to/asl_dir", "Output EuRoC/ASL directory"),
]
VALUE_ARGS = [
    ("multiple_files", "false", "Concatenate all videos in gopro_folder"),
    ("scale", "0.5", "Image scaling factor"),
    ("grayscale", "true", "Convert images to grayscale"),
    ("display_images", "false", "Show images while writing"),
    ("hardware_decoding", "true", "Decode on the GPU (NVDEC/VAAPI) if available"),
]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in STRING_ARGS + VALUE_ARGS
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=str)
        for name, _, _ in STRING_ARGS
    }
    parameters.update({name: LaunchConfiguration(name) for name, _, _ in VALUE_ARGS})

    node = Node(
        package="gopro_ros",
        executable="gopro_to_asl",
        name="gopro_to_asl",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription(args + [node])
