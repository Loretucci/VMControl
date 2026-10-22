import rclpy
from std_msgs.msg import String

rclpy.init()
node2 = rclpy.create_node("my_subscriber")

def callback(messaggio):
    node2.get_logger().info(f"Ricevuto: {messaggio.data}")

subscriber = node2.create_subscription(String, "chatter", callback, 10) # 10 is the length of the messages queue



rclpy.spin(node2)           #it keeps the node alive

node2.destroy_node()
rclpy.shutdown()

