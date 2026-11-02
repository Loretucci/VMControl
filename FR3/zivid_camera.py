import numpy as np
import zivid
import rclpy

from pathlib import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from time import monotonic, sleep
from rclpy.duration import Duration


def aruco_pose_crf(frame, marker_dict, marker_id_number):
    '''It returns the transformation matrix between ARuco marker frame and camera reference frame
        
        Inputs: frame, marker_dictionary, marker_id
        Outputs: transformation matrix '''

    print("Configuring ArUco marker")
    marker_id = [marker_id_number]      # da mettere il nostro id

    print("Detecting ArUco marker")
    detection_result = zivid.calibration.detect_markers(frame, marker_id, marker_dict)
    if not detection_result.valid():
        raise RuntimeError("ArUco non rilevato nel frame")


    print("Estimating pose of detected ArUco marker")
    T_camera_aruco = detection_result.detected_markers()[0].pose.to_matrix() # Transformation matrix between aruco marker reference frame and CRF

    return T_camera_aruco


def aruco_pose(T_base_camera, T_camera_aruco):
    '''It returns Aruco position' in metres and orientation
    Inputs: - Transformation matrix between base and camera
            - Transformation matrix between camera and Aruco
    Output: Aruco position wrt base frame 
    '''
    T_base_aruco = T_base_camera @ T_camera_aruco
    p_base_m = T_base_aruco[:3, 3] / 1000
    R_base_aruco = T_base_aruco[:3, :3]

    return p_base_m, R_base_aruco

def camera_publisher():
    rclpy.init()
    node = rclpy.create_node("my_publisher")
    publisher = node.create_publisher(PoseStamped, "/vision/aruco_pose", 1) # 1 is the length of the messages queue
    node.get_logger().info("Nodo avviato")
    return node, publisher

def send_pose(node, publisher, aruco_pos, aruco_rot):

    message = PoseStamped()
    message.header.frame_id = "base"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = float(aruco_pos[0])
    message.pose.position.y = float(aruco_pos[1])
    message.pose.position.z = float(aruco_pos[2])

    q = Rotation.from_matrix(aruco_rot).as_quat()  # ordine x, y, z, w
    message.pose.orientation.x = float(q[0])
    message.pose.orientation.y = float(q[1])
    message.pose.orientation.z = float(q[2])
    message.pose.orientation.w = float(q[3])

    publisher.publish(message)
    node.get_logger().info(f"Pubblicato ArUco: p={aruco_pos}, q={q}")

def main():
    app = zivid.Application()

    camera = app.connect_camera()
    settings = zivid.Settings.load("percorso/delle/impostazioni.yml")

    marker_id = 16
    marker_dict = zivid.calibration.MarkerDictionary.aruco5x5_1000
    frame = camera.capture_2d_3d(settings)

    T_camera_aruco = aruco_pose_crf(frame,marker_dict, marker_id)
    T_base_camera = np.load(
        "/home/lorenzoantonio/zivid_ws/calib_2026-10-21/calibration_outputs/T_cam_to_base_Zivid.npy"
    )
    aruco_pos, aruco_rot  = aruco_pose(T_base_camera, T_camera_aruco)
    
    node, publisher = camera_publisher()
    try:
        # Waiting for someone to be connected to the topic
        while publisher.get_subscription_count() == 0:
            sleep(0.1)

        send_pose(node, publisher, aruco_pos, aruco_rot)
        publisher.wait_for_all_acked(Duration(seconds=2.0))

    finally:
    
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


#source /opt/ros/humble/setup.bash
#ros2 topic echo /vision/aruco_pose geometry_msgs/msg/PoseStamped
