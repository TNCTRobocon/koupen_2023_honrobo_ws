import sys
import typing
from PyQt5.QtCore import Qt,QTimer
from PyQt5.QtWidgets import QDialog, QApplication, QWidget
from qi_ros_test import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import time

class Ros_Gui(Node):
    node_name = "gui_pub"
    topic_name = "chatter"
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.pub = self.create_publisher(String,self.topic_name,10)
        self.msg = String()
        
    def send_entry(self,rcv_msg):
        self.msg.data = str(rcv_msg)
        self.pub.publish(self.msg)

class MyForm(QDialog):
    def __init__(self, parent=None):
        super(MyForm,self).__init__(parent)
        
        rclpy.init(args=None)
        self.ros_gui = Ros_Gui()
        
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui.pushButton.clicked.connect(self.put_text)
        self.ui.pushButton_2.clicked.connect(self.put_text2)
        self.show()
        
    def put_text(self):
        self.ui.label.setText("test")
        self.ros_gui.send_entry("test")
        
    def put_text2(self):
        self.ui.label.setText("test2")
        self.ros_gui.send_entry("test2")
        

        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MyForm()
    w.show()
    sys.exit(app.exec_())