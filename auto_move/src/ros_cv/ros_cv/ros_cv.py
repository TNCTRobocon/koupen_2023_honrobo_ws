import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,Joy,CompressedImage
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import math
import numpy as np

from .module.JoyCalcTools import JoyCalcTools

class RosCv(Node):
    global joy_data_r,cv_image
    node_name = 'ros_cv'
    sub_joy_topic_name = 'joy'
    # sub_img_topic_name = 'image_topic_compressed' # 圧縮時のトピック名
    sub_img_topic_name = "image_topic"
    
    pub_joy_topic_name = 'converted_joy'
    pub_img_topic_name = 'image_result'
    axis_max = 50 # どこまで操作するかパラメータ
    arrow_fruits_lost = 30 # 何フレーム検出飛びを許すか
    none_detect_count = 0 # 記録用変数
    

    
    def __init__(self):
        global joy_data_r,cv_image
        super().__init__(self.node_name)
        
        self.sub = self.create_subscription(Joy,self.sub_joy_topic_name,self.joy_callback,10)
        self.sub2 = self.create_subscription(Image,self.sub_img_topic_name,self.image_callback,10)
        
        self.pub = self.create_publisher(Joy,self.pub_joy_topic_name,10)
        self.pub2 = self.create_publisher(Image,self.pub_img_topic_name,10)
        self.joy_msg = Joy()
        self.img = Image()
        self.bridge = CvBridge()

        self.model = YOLO("auto_move/src/ros_cv/ros_cv/weights/best.pt")
        self.joy_tool = JoyCalcTools(1)
        
    def joy_callback(self,data):
        global joy_data_r
        joy_data_r = data
        
    def image_callback(self,image):
        global cv_image
        
        # 圧縮あり1
        # self.np_arr = np.frombuffer(image.data, np.uint8)
        # cv_image = cv2.imdecode(self.np_arr, cv2.IMREAD_COLOR)
        
        # 圧縮あり2
        # cv_image = self.bridge.compressed_imgmsg_to_cv2(image)
        
        # 圧縮しない
        cv_image = self.bridge.imgmsg_to_cv2(image)
        self.mainprocess(0) # 0: auto_ON 1:auto_OFF
        
    def mainprocess(self,mode):
        global joy_data_r,cv_image
        
        try: #Joyデータを受信していなかった際に待機(処理を飛ばす)
            if mode:
                self.publish_joy(joy_data_r,frame)
            if joy_data_r:
                frame, results = self.detect_fruits(cv_image)
                dis, per, pm = self.calc_points(frame,results)
                ans = self.calc_move(dis,per)
                recalc_joy = self.override_side_joy(joy_data_r,ans,pm)
                self.publish_joy(recalc_joy,frame)
        except:
            pass
        

    def detect_fruits(self,img):
        results = self.model.track(source=img, tracker="botsort.yaml", conf=0.7, iou=0.7, persist=True) # 検出&トラッキング
        return img,results
    
    def calc_points(self,frame,results):
        bbox = results[0].boxes.xyxy
        bbox_np = bbox.to('cpu').detach().numpy().copy()
        
        self.frame_y, self.frame_x, _ = frame.shape
        self.frame_centor_x = int(self.frame_x * 0.5)
        self.frame_centor_y = int(self.frame_y * 0.5)
        self.distance_max = self.frame_centor_x
        
        cv2.line(frame, (self.frame_centor_x, 0), (self.frame_centor_x, self.frame_y),(0, 255, 0),thickness=2) # 中心線
        if bbox_np.size:
            self.none_detect_count = 0
            x1 = int(bbox_np[0][0])
            y1 = int(bbox_np[0][1])
            x2 = int(bbox_np[0][2])
            y2 = int(bbox_np[0][3])
            centor_x = int((x1 + x2) * 0.5)
            centor_y = int((y1 + y2) * 0.5)
            helf_x1 = int((x1 + centor_x)*0.5)
            helf_x2 = int((x2 + centor_x)*0.5)
            
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0))
            cv2.rectangle(frame, (helf_x1,y1), (helf_x2,y2), (0,255,0)) 
            cv2.line(frame, (self.frame_centor_x, centor_y), (centor_x, centor_y),(0, 255, 0),thickness=2)
            
            self.distance_of_two = abs(self.frame_centor_x - centor_x)
            self.per_of_frame = ((x2-x1) * (y2-y1)) / (self.frame_x * self.frame_y)
            

            if centor_x < self.frame_centor_x:
                self.direction = -1
            else :
                self.direction = 1

            print(self.none_detect_count)
            
            if helf_x1 < self.frame_centor_x and self.frame_centor_x < helf_x2:
                return 0, 0, 0
            return self.distance_of_two, self.per_of_frame, self.direction
        
        if self.none_detect_count > self.arrow_fruits_lost:
            self.none_detect_count += 1
            print(self.none_detect_count)
            return 0, 0, 0
        self.none_detect_count += 1
        try:
            print(self.none_detect_count)
            return self.distance_of_two, self.per_of_frame, self.direction
        except:
            print(self.none_detect_count)
            return 0, 0, 0
    
    def calc_move(self,distance,percentage):
        distance_percent = distance / self.distance_max
        percentage = 1 - percentage
        return distance_percent * percentage 

    
    def override_side_joy(self,joy,ans,direc):
        target_joy = joy
        diff = ans * self.axis_max * direc / 127
        clamp= lambda x: min(1, max(x, -1))
        diff = target_joy.axes[2] + diff
        
        target_joy.axes[2] = clamp(diff)
        return target_joy
    
    def publish_joy(self,joy,frame):
        self.joy_msg = joy
        self.pub.publish(self.joy_msg)

        imgs = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.img = imgs
       # self.pub2.publish(self.img)
        
        
 
 
def main(args=None):
    rclpy.init(args=args)
    ros_cv = RosCv()
    
    try:
        rclpy.spin(ros_cv)
    except KeyboardInterrupt:
        pass
    finally:
        ros_cv.destroy_node()
        
if __name__ == '__main__':
    main()

    
    
    
        
