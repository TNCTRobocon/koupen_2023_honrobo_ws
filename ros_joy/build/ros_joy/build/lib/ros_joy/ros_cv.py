import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,Joy
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import math

from .module.JoyCalcTools  import JoyCalcTools

class RosCv(Node):
    node_name = 'ros_cv'
    sub_topic_name = 'joy'
    pub_joy_topic_name = 'converted_joy'
    pub_img_topic_name = 'converted_img'
    axis_max = 50
    
    def __init__(self):
        super().__init__(self.node_name)
        
        self.sub = self.create_subscription(Joy,self.sub_topic_name,self.callback,10)
        self.pub = self.create_publisher(Joy,self.pub_joy_topic_name,10)
        self.pub2 = self.create_publisher(Image,self.pub_img_topic_name,10)
        self.joy_msg = Joy()
        self.img = Image()
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,360)
        self.model = YOLO('./runs/detect/train/weights/best.pt')
        self.joy_tool = JoyCalcTools(1)
        
    def callback(self,data):
        frame, results = self.detect_fruits(data)
        dis, per, pm = self.calc_points(frame,results)
        ans = self.calc_move(dis,per)
        recalc_joy = self.override_side_joy(data,ans,pm)
        self.publish_joy(recalc_joy,frame)
        

    def detect_fruits(self,data):
        ret, frame = self.cap.read()
        if not ret:
            return
        results = self.model.track(source=frame, tracker="botsort.yaml", conf=0.5, iou=0.5)
        return frame,results
    
    def calc_points(self,frame,results):
        bbox = results[0].boxes.xyxy
        bbox_np = bbox.to('cpu').detach().numpy().copy()
        
        frame_y, frame_x, _ = frame.shape
        self.frame_centor_x = int(frame_x * 0.5)
        frame_centor_y = int(frame_y * 0.5)
        # self.distance_max = math.sqrt((self.frame_centor_x ** 2) + (frame_centor_y ** 2))
        self.distance_max = self.frame_centor_x

        if bbox_np.size:
            x1 = int(bbox_np[0][0])
            y1 = int(bbox_np[0][1])
            x2 = int(bbox_np[0][2])
            y2 = int(bbox_np[0][3])
            centor_x = int((x1 + x2) * 0.5)
            centor_y = int((y1 + y2) * 0.5)
            
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0))
            cv2.drawMarker(frame, (centor_x, centor_y), color=(0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=4, thickness=2)   
            cv2.line(frame, (self.frame_centor_x, centor_y), (centor_x, centor_y),(0, 255, 0),thickness=2)
            
            # distance_of_two = math.sqrt((self.frame_centor_x - centor_x) ** 2 + (frame_centor_y - centor_y) ** 2)
            distance_of_two = abs(self.frame_centor_x - centor_x)
            per_of_frame = ((x2-x1) * (y2-y1)) / (frame_x * frame_y)
            
            # pm = -1 if centor_x < self.frame_centor_x else 1
            if centor_x < self.frame_centor_x:
                pm = -1
            else :
                pm = 1
            
            # cv2.imshow("video",frame)
            # cv2.waitKey(1)
            
            return distance_of_two, per_of_frame, pm
        
        # cv2.imshow("video",frame)
        # cv2.waitKey(1)
        return 0, 0, 0
    
    def calc_move(self,distance,percentage):
        distance_percent = distance / self.distance_max
        percentage = 1 - percentage
        return distance_percent * percentage 
    
    def override_side_joy(self,joy,ans,pm):
        target_joy = joy
        diff = ans * self.axis_max * pm / 127
        clamp= lambda x: min(1, max(x, -1))
        diff = target_joy.axes[2] + diff
        
        target_joy.axes[2] = clamp(diff)
        print(target_joy.axes[2])
        return target_joy
    
    def publish_joy(self,joy,frame):
        self.joy_msg = joy
        self.pub.publish(self.joy_msg)
        bridge = CvBridge()
        imgs = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.img = imgs
        self.pub2.publish(self.img)
        
        
 
 
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

    
    
    
        
