import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RcvNode(Node):
    node_name = 'rcv_node'
    topic_name = 'chatter'
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.sub = self.create_subscription(String,self.topic_name,self.callback,10)
        
    def callback(self,msg):
        print("{}".format(msg))
        
def main(args=None):
    rclpy.init(args=args)
    rcvnode = RcvNode()
    
    try:
        rclpy.spin(rcvnode)
    except KeyboardInterrupt:
        pass
    finally:
        rcvnode.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()