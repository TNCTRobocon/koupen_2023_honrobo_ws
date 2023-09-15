import sys
import typing
from PyQt5 import QtCore,QtGui
from PyQt5.QtCore import Qt,QTimer,QThread,pyqtSignal,pyqtSlot
from PyQt5.QtWidgets import QDialog, QApplication, QWidget
from ui2 import *
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image


import time

class VideoThread(QThread):
    change_signal = pyqtSignal(np.ndarray)
    
    def __init__(self):
        super().__init__()
        self._run_flag = True
        
    def run(self,cap):
        if cap != None:
            self.change_signal.emit(cap)
            
    def __del__(self):
        self._run_flag = False
        self.wait()
   

class Video_ROS_Thread(Node):
    def __init__(self):
        self.node_name = "Qt_UI1"
        self.vid_topic_name = "image_raw"
        self.data_topic_name = ""
        self.create_subscription(Image,self.vid_topic_name,self.callback,10)
        self.vid_th = VideoThread()
        self.cvt = ImageConverter()
        
    def callback(self,msg):
        cv_image = self.cvt.cvt(msg)
        self.vid_th.run(cv_image)
        
    def connect_ros(self,state):
        if state:
            try:
                rclpy.init(args=None)
                timeout_sec = 5
                timeout_init = time.time()
                rclpy.spin_once(Video_ROS_Thread,timeout_sec)
                timeout_end = time.time()
                ros_cnt_time = timeout_end - timeout_init
                if ros_cnt_time >= timeout_sec:
                    print("Couldnt connect")
                else:
                    print("connected")
            except :
                pass
            
        else:
            self.destroy_node()
            rclpy.shutdown()
    
    def update_ros(self,state):
        if state:
            self.timer = QTimer(QDialog)
            self.timer.timeout.connect(self.timer_update)
            
            self.timer.start(10)
        else:
            self.timer.stop()
            
            
class ImageConverter():
    def __init__(self):
        self.bridge = CvBridge()
    
    def cvt(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        return cv_image
            

class MainWindow(QDialog):
    def __init__(self,parent=None):
        super(MainWindow,self).__init__(parent)
        self.scene = QtWidgets.QGraphicsScene
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.show()
        
        self.thread = Video_ROS_Thread()
        self.thread.vid_th.change_signal.connect(self.update_image)
        self.thread.start()
        
    def closeEvent(self,event):
        self.thread.vid_th.__del__()
        event.accept()
        
    @pyqtSlot(np.ndarray)
    def update_image(self,img):
        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        height, width, dim = img.shape
        bytesPerLine = dim * width
        self.image = QtGui.QImage(img.data,width,height,bytesPerLine,QtGui.QImage.Format_RGB888)
        self.item = QtWidgets.QGraphicsItem(QtGui.QPixmap.fromImage(self.image))
        self.scene.addItem(self.item)
        self.ui.graphicsView.setScene(self.scene)
        
        self.ui.graphicsView.show()
        
        
if __name__ == "__main__":
    