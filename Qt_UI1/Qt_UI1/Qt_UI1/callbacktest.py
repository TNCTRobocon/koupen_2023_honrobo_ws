import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage

class Nodes(Node):
    node_name = "test_node"
    topic_name = "image_raw/compressed"
    def __init__(self):
        super().__init__(self.node_name)
        self.create_subscription(CompressedImage,self.topic_name,self.callback,10)
        
    def callback(self,data):
        print("callbacked")
        
if __name__ == "__main__":
    rclpy.init(args=None)
    nodes = Nodes()
    