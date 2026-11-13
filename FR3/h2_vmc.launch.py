"""Launch the gripper on the PC2 host or H2 in the VMC environment.

Run this file once with start_h2:=false in the host's gripper Python environment.
After the gripper is open and the camera is publishing, run it again with
start_h2:=true in the VMC shell. The two runs never start the other process.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_selected_process(context, script_dir):
    """Turn launch arguments into one process command in the current shell."""
    python_executable = LaunchConfiguration("python_executable").perform(context)
    start_h2 = LaunchConfiguration("start_h2").perform(context).lower()

    if start_h2 == "false":
        # This branch is intended for the PC2 host venv that has pymodbus and
        # access to the gripper's USB serial adapter.
        gripper_port = LaunchConfiguration("gripper_port").perform(context)
        command = [
            python_executable,
            str(script_dir / "gripper_node.py"),
            "--ros-args", "-p", f"port:={gripper_port}",
        ]
    elif start_h2 == "true":
        # This branch is intended for /opt/venv in the VMC container. Default
        # torques go to an isolated topic until a real controller topic is
        # explicitly selected at launch time.
        command = [python_executable, str(script_dir / "h2_vmc_ros.py")]
        h2_options = (
            ("sub_topic", "--sub-topic"),
            ("torque_pub_topic", "--torque-pub-topic"),
            ("camera_topic", "--camera-topic"),
            ("pub_rate", "--pub-rate"),
            ("link_length", "--link-length"),
            ("stiffness", "--stiffness"),
            ("damping", "--damping"),
            ("max_force", "--max-force"),
            ("timestep", "--timestep"),
            ("state_timeout", "--state-timeout"),
            ("duration", "--duration"),
        )
        for launch_name, script_option in h2_options:
            command.extend((script_option, LaunchConfiguration(launch_name).perform(context)))

        headless = LaunchConfiguration("headless").perform(context).lower()
        if headless == "true":
            command.append("--headless")
        elif headless != "false":
            raise ValueError("headless must be true or false")
    else:
        raise ValueError("start_h2 must be true or false")

    return [ExecuteProcess(cmd=command, cwd=str(script_dir), output="screen")]


def generate_launch_description():
    script_dir = Path(__file__).resolve().parent

    return LaunchDescription([
        DeclareLaunchArgument(
            "python_executable",
            default_value="python3",
            description="Python of the host gripper venv or the VMC container venv",
        ),
        DeclareLaunchArgument(
            "start_h2",
            default_value="false",
            description="false: gripper only on the host; true: H2 only in the VMC shell",
        ),
        DeclareLaunchArgument("gripper_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("sub_topic", default_value="/robot2/franka/joint_states"),
        DeclareLaunchArgument("torque_pub_topic", default_value="/vmc/h2_dry_run_torques"),
        DeclareLaunchArgument("camera_topic", default_value="/vision/aruco_pose"),
        DeclareLaunchArgument("pub_rate", default_value="1000.0"),
        DeclareLaunchArgument("link_length", default_value="0.4"),
        DeclareLaunchArgument("stiffness", default_value="2000.0"),
        DeclareLaunchArgument("damping", default_value="50.0"),
        DeclareLaunchArgument("max_force", default_value="20.0"),
        DeclareLaunchArgument("timestep", default_value="0.001"),
        DeclareLaunchArgument("state_timeout", default_value="0.2"),
        DeclareLaunchArgument("duration", default_value="0"),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="true: run H2 without the MuJoCo viewer",
        ),
        OpaqueFunction(function=_launch_selected_process, args=[script_dir]),
    ])
