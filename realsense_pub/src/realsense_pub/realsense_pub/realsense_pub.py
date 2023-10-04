import pyrealsense2 as rs
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RsPub(Node):
    node_name = "rs_pub"
    image_topic = "image_topic"
    depth_topic = "depth_topic"
    
    def __init__(self):
        super().__init__(self.node_name)
        
        self.image_pub = self.create_publisher(Image,self.image_topic,10)
        self.depth_pub = self.create_publisher(Image,self.depth_topic,10)
        
        self.img_msg = Image()
        self.depth_msg = Image()
        self.bridge = CvBridge()
        
        config = rs.config()
        config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,60)
        config.enable_stream(rs.stream.depth,640,480,rs.format.z16,60)

        self.pipeline = rs.pipeline()
        profile = self.pipeline.start(config)

        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        RGB_frame = aligned_frames.get_color_frame()
        RGB_image = np.asanyarray(RGB_frame.get_data())
        
        depth_frame = aligned_frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,alpha=0.08),cv2.COLORMAP_JET)
        
        RGB_image_s = cv2.resize(RGB_image, (640,480))
        depth_colormap_s = cv2.resize(depth_colormap,(640,480))
        
        return RGB_image_s, depth_colormap_s
    
    def pub_frame(self,image,depth):
        image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.img_msg = image
        self.image_pub.publish(self.img_msg)
        depth = self.bridge.cv2_to_imgmsg(depth, encoding="bgr8")
        self.depth_msg = depth
        self.depth_pub.publish(self.depth_msg)
        
    def publisher(self):
        img, dpth = self.get_frame()
        self.pub_frame(img,dpth)
        
        
        
def main(args = None):
    rclpy.init(args=args)
    rs_pub = RsPub()
    while 1:
        rs_pub.publisher()
    rclpy.spin(rs_pub)
    rs_pub.destroy_node()
    
    
if __name__ == '__main__':
    main()