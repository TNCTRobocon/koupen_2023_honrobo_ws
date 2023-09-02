import sys
import typing
from PyQt5 import QtCore
from PyQt5.QtCore import Qt,QTimer
from PyQt5.QtWidgets import QDialog, QApplication, QWidget
from gui_sub_ui import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import time
       
class MyForm(QDialog):
    def __init__(self, parent=None):
        node_name = "gui_sub"
        topic_name = "chatter"
        super(MyForm,self).__init__(parent)
        
        rclpy.init(args=None)
        self.ros_gui = Node(node_name)
        self.sub = self.ros_gui.create_subscription(
            String,
            topic_name,
            self.callback,
            10
        )
        
        self.ui = Ui_Form
        self.ui.setupUi(self)
        
        self.show()
        
    def connect_ros(self,state):
        if state:
            try:
                timeout_sec_rclpy = 5
                timeout_init = time.time()
                rclpy.spin_once(self.ros_gui, timeout_sec=timeout_sec_rclpy)
                timeout_end = time.time()
                ros_connect_time = timeout_end - timeout_init
                
                if ros_connect_time >= timeout_sec_rclpy:
                    print("Couldn't connect")
                else:
                    print("Connected")
            except:
                pass
        else:
            self.ros_gui.destroy_node()
            rclpy.shutdown()
            
    def update_ros(self,state):
        if state:
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.ros_update)
            self.timer.start(10)
        else:
            self.timer.stop()
            
    def callback(self,msg):
        
            
    