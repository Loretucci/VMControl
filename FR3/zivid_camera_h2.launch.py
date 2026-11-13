"""Start the H2 ArUco camera publisher in the PC2 host's Zivid environment."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    script_dir = Path(__file__).resolve().parent

    return LaunchDescription([
        DeclareLaunchArgument(
            "python_executable",
            default_value="python3",
            description="Python interpreter with Zivid, ROS 2 and SciPy installed",
        ),
        DeclareLaunchArgument(
            "camera_workdir",
            default_value="/home/lorenzoantonio",
            description="Working directory used to resolve the camera script's relative settings path",
        ),
        ExecuteProcess(
            cmd=[LaunchConfiguration("python_executable"), str(script_dir / "zivid_camera_h2.py")],
            cwd=LaunchConfiguration("camera_workdir"),
            output="screen",
        ),
    ])
