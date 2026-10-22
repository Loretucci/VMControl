import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("my_publisher")
publisher = node.create_publisher(String, "chatter", 10) # 10 is the length of the messages queue
node.get_logger().info("Nodo avviato")

def invia_messaggio():
    messaggio = String()
    messaggio.data = "Ciao da ROS 2"
    publisher.publish(messaggio)
    node.get_logger().info(f"Pubblicato: {messaggio.data}")

node.create_timer(1.0, invia_messaggio) #it publishes every 1 second
rclpy.spin(node)           #it keeps the node alive

node.destroy_node()
rclpy.shutdown()