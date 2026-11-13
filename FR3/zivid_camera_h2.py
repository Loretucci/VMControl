"""Continuously publish one detected ArUco marker pose in the FR3 base frame."""

import numpy as np
import zivid
import rclpy

from pathlib import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from time import monotonic, sleep
from rclpy.duration import Duration

# Inactive discovery helper from the initial camera setup. It can identify an
# unknown marker dictionary and ID on a single frame; the running node below
# deliberately uses one configured dictionary and ID instead.
# def scan_markers(frame):
#     """Try every ArUco dictionary and every ID on one frame; print what is found.
#     Used once to discover which dictionary/ID the lab cubes use.
#     Note: dictionaries overlap (4x4_50 is contained in 4x4_100, ...): pick the smallest one that works.
#     """
#     MarkerDictionary = zivid.calibration.MarkerDictionary
#     found_any = False

#     for dictionary in MarkerDictionary.valid_values():
#         ids = list(range(MarkerDictionary.marker_count(dictionary)))
#         result = zivid.calibration.detect_markers(frame, ids, dictionary)
#         markers = result.detected_markers()
#         for marker in markers:
#             found_any = True
#             p_C = np.asarray(marker.pose.to_matrix())[:3, 3]  # marker center in camera frame [mm]
#             print(f"{dictionary:>14s}  id={marker.identifier:<4d}  p_C [mm] = {np.round(p_C, 1)}")

#     if not found_any:
#         print("Scan: no marker found in any dictionary")
        

def aruco_pose_crf(frame, marker_dict, marker_id_number):
    """Return camera_T_marker in millimetres, or None when detection fails.

    The Zivid frame and the requested marker dictionary/ID are the inputs.
    The pose maps coordinates expressed in the marker frame to the camera
    frame. An unsuccessful detection is expected during normal scanning.
    """

    # detect_markers expects a collection of IDs, even when tracking only one.
    print("Configuring ArUco marker")
    marker_id = [marker_id_number]

    print("Detecting ArUco marker")
    detection_result = zivid.calibration.detect_markers(frame, marker_id, marker_dict)
    # No pose can be transformed or published for this frame if the requested
    # marker was not detected; the caller will acquire a new frame.
    if not detection_result.valid():
        return None

    # Only the configured marker is requested, so the first detected pose is
    # the marker pose used by this node.
    print("Estimating pose of detected ArUco marker")
    T_camera_aruco = detection_result.detected_markers()[0].pose.to_matrix()

    return T_camera_aruco


def aruco_pose(T_base_camera, T_camera_aruco):
    """Express the marker pose in the FR3 base frame.

    Both transforms use the convention destination_T_source. Their product
    maps marker-frame coordinates into the robot base frame. Zivid matrices
    use millimetres, while the ROS PoseStamped position must be in metres.
    """
    T_base_aruco = T_base_camera @ T_camera_aruco
    # Convert translation only; the 3x3 rotation has no length unit.
    p_base_m = T_base_aruco[:3, 3] / 1000
    R_base_aruco = T_base_aruco[:3, :3]

    return p_base_m, R_base_aruco

def camera_publisher():
    """Create the ROS node and the pose publisher once for the whole run."""
    rclpy.init()
    node = rclpy.create_node("my_publisher")
    publisher = node.create_publisher(PoseStamped, "/vision/aruco_pose", 1) # Queue depth: retain the newest pose.
    node.get_logger().info("Nodo avviato")
    return node, publisher

def send_pose(node, publisher, aruco_pos, aruco_rot):
    """Publish a base-frame marker pose with the current ROS timestamp."""

    message = PoseStamped()
    # h2_vmc_ros.py accepts camera messages only when frame_id is "base".
    message.header.frame_id = "base"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = float(aruco_pos[0])
    message.pose.position.y = float(aruco_pos[1])
    message.pose.position.z = float(aruco_pos[2])

    # SciPy returns [x, y, z, w], matching geometry_msgs/Quaternion.
    q = Rotation.from_matrix(aruco_rot).as_quat()
    message.pose.orientation.x = float(q[0])
    message.pose.orientation.y = float(q[1])
    message.pose.orientation.z = float(q[2])
    message.pose.orientation.w = float(q[3])

    publisher.publish(message)
    node.get_logger().info(f"Pubblicato ArUco: p={aruco_pos}, q={q}")

def main():
    """Load camera resources once, then detect and publish on every frame."""
    # Keep the Zivid application and camera alive across the full capture loop.
    app = zivid.Application()

    camera = app.connect_camera()
    # This path is relative to the process working directory in the Zivid
    # container. The launch configuration must preserve that directory.
    settings = zivid.Settings.load("Scrivania/Tesi/First_Ex_LAB_VMC_OmarLibrary_27-10/zivid_camera_settings.yml")

    # Track the chosen physical marker. Both values must match the printout.
    marker_id = 16
    marker_dict = zivid.calibration.MarkerDictionary.aruco5x5_1000

    # Fixed camera-to-base calibration from the lab calibration session.
    T_base_camera = np.load(
        "/home/lorenzoantonio/zivid_ws/calib_2026-10-21/calibration_outputs/T_cam_to_base_Zivid.npy"
    )
    
    node, publisher = camera_publisher()
    try:
        # ROS publisher-only operation does not need rclpy.spin(): each new
        # frame is handled synchronously, and publish() sends its pose.
        while rclpy.ok():
            # A new capture on every iteration prevents a stale marker pose
            # from being republished as a fresh observation.
            frame = camera.capture_2d_3d(settings)
            T_camera_aruco = aruco_pose_crf(frame,marker_dict, marker_id)

            # No marker in this frame: retry with the next capture. This also
            # lets the controller's camera timeout notice missing detections.
            if T_camera_aruco is None:
                continue

            # Publish only a successfully detected, base-frame marker pose.
            aruco_pos, aruco_rot  = aruco_pose(T_base_camera, T_camera_aruco)
            send_pose(node, publisher, aruco_pos, aruco_rot)

    finally:
        # Release the ROS node even when acquisition raises an exception or
        # the loop is interrupted.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


