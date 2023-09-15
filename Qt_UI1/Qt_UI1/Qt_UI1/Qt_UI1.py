from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap,QImage

from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread

import cv2 as cv
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage


class App(QWidget):

   def __init__(self):
       super().__init__()

       self.image_label = QLabel(self)
       
       # vboxにQLabelをセット
       vbox = QVBoxLayout()
       vbox.addWidget(self.image_label)

       # vboxをレイアウトとして配置
       self.setLayout(vbox)

   def update_image(self, cv_img):
       #img = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)
       # QT側でチャネル順BGRを指定
       qimage = QImage.fromData(cv_img.data)
       self.image_label.setPixmap(QPixmap.fromImage(qimage))
       
    #    qimg = QtGui.QImage(cv_img.tobytes(),cv_img.shape[1],cv_img.shape[0],cv_img.strides[0],QtGui.QImage.Format.Format_BGR888)
    #    qpix = QPixmap.fromImage(qimg)
    #    self.image_label.setPixmap(qpix)
       
class Video_Node(Node):
    node_name = "Qt_UI1"
    sub_topic_name = "/image_raw/compressed"
    def __init__(self):
        super().__init__(self.node_name)
        self.sub = self.create_subscription(CompressedImage,self.sub_topic_name,self.callback,10)
        self.app = QApplication([])
        self.window = App()
        print("setup")
        
    def callback(self,data):
        print("aaa")
        self.window.update_image(data)
        
def do_thread1(node):
    node.window.show()
    node.app.exec_()
def do_thread2(node):
    rclpy.spin(node)
    
        

if __name__ == "__main__":
    rclpy.init(args=None)
    vid_node = Video_Node()
    thread1 = threading.Thread(target=do_thread1,args=(vid_node,))
    thread2 = threading.Thread(target=do_thread2,args=(vid_node,))
    
    thread1.start()
    thread2.start()
    
      
        

# if __name__ == "__main__":
#     rclpy.init(args=None)
#     vid_node = Video_Node()
#     vid_node.window.show()
    
#     vid_node.app.exec_()
    
    