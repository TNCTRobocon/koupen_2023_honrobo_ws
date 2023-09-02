import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import can
from UsbCan import UsbCan
import time

class JoyCan(Node):
    node_name = 'joy_can'
    topic_name = 'joy'
    Ucan = UsbCan()
    send_id1 = [0] * 8
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.sub = self.create_subscription(Joy,self.topic_name,self.callback,10)
        self.Ucan.open()
        self.get_logger().info("setup done")
    def __del__(self):
        self.Ucan.close()
        
    def callback(self,joy):
        self.send_id1[0] = joy.axes[0] * 127 + 128
        self.send_id1[1] = joy.axes[1] * 127 + 128
        self.send_id1[2] = joy.axes[3] * -127 + 128
        self.send_id1[3] = joy.axes[4] * -127 + 128
        self.send_id1[4] = joy.axes[2] * 127 + 128
        self.send_id1[5] = joy.axes[5] * 127 + 128
        self.send_id1[6] = joy.axes[6] + 1
        self.send_id1[7] = joy.axes[7] + 1
        
        self.send_id1 = list(map(int,self.send_id1))
        msg1 = can.Message(
            arbitration_id=0x001,
            is_extended_id=False,
            data=[int(self.send_id1[0]),int(self.send_id1[1]),int(self.send_id1[2]),int(self.send_id1[3]),int(self.send_id1[4]),int(self.send_id1[5]),int(self.send_id1[6]),int(self.send_id1[7])]
            #data=send_id1
        )
        
        self.Ucan.send(msg1)
        print_text = "axis:{},{},{},{},{},{},{},{}"
        self.get_logger().info(print_text.format(*self.send_id1))
        
        time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    joy_can = JoyCan()
    try:
        rclpy.spin(joy_can)
    except KeyboardInterrupt:
        pass
    finally:
        joy_can.destroy_node()

if __name__ == '__main__':
    main()
