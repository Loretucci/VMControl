"""Start one H2 process at a time inside the VMC container.

Run the default mode first: it starts gripper_node.py, including its initial
activation and opening motion. After the gripper is fully open and the camera
is publishing, run this launch file again with start_h2:=true in another VMC
shell. That second invocation starts only h2_vmc_ros.py.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    script_dir = Path(__file__).resolve().parent
    python_executable = LaunchConfiguration("python_executable")
    start_h2 = LaunchConfiguration("start_h2")

    return LaunchDescription([
        DeclareLaunchArgument(
            "python_executable",
            default_value="python3",
            description="Python interpreter available in the VMC container",
        ),
        DeclareLaunchArgument(
            "start_h2",
            default_value="false",
            description="false: start the gripper; true: start H2 only after gripper and camera are ready",
        ),
        ExecuteProcess(
            cmd=[python_executable, str(script_dir / "gripper_node.py")],
            cwd=str(script_dir),
            output="screen",
            condition=UnlessCondition(start_h2),
        ),
        ExecuteProcess(
            cmd=[python_executable, str(script_dir / "h2_vmc_ros.py"), "--duration", "0"],
            cwd=str(script_dir),
            output="screen",
            condition=IfCondition(start_h2),
        ),
    ])
