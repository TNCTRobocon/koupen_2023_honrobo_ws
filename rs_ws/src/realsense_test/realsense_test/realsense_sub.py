import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as rosImage

import cv2
from cv_bridge import CvBridge,CvBridgeError


class RsSub(Node):
    node_name1 = 'realsense_sub'
    topic_name1 = '/camera/depth/image_rect_raw'
    
    def __init__(self) -> None:
        super().__init__(self.node_name1)
        self.sub = self.create_subscription(rosImage,self.topic_name1,self.callback,10)
        self.bridge = CvBridge()
        cv2.namedWindow('RealSense',cv2.WINDOW_AUTOSIZE)
        
    def __del__(self):
        pass
    
    def callback(self,image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image,"bgr8")
            cv2.imshow('RealSense',self.cv_image)
            cv2.waitKey(1)
            
            
        except CvBridgeError as e:
            print(e)
        
        finally:
            pass

def main():
    rclpy.init()
    Realsense_sub = RsSub()
    try:
        rclpy.spin(Realsense_sub)
    except KeyboardInterrupt:
        pass
    finally:
        RsSub.destroy_node

if __name__ == '__main__':
    main()

