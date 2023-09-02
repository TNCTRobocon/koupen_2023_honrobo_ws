import rclpy
from rclpy.node import Node

import cv2

from sensor_msgs.msg import Image as rosImage 
from cv_bridge import CvBridge,CvBridgeError

import tkinter as tk
from PIL import Image,ImageTk

import threading

class ImageConverter(Node):
    node_name = 'image_converter'
    #pub_topic = 'image_topic'
    sub_topic = 'image_raw'
    
    def __init__(self):
        super().__init__(self.node_name)
        #self.pub = self.create_publisher(Image,self.pub_topic,10)
        self.sub = self.create_subscription(rosImage,self.sub_topic,self.callback,10)
        self.bridge = CvBridge()
        self.root = tk.Tk()
        self.app = Application(master = self.root)
        
    def callback(self,data):
        global cv_image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        self.app.disp_image(cv_image)
        
class Application(tk.Frame):
    def __init__(self,master = None):
        super().__init__(master)
        self.pack()
        
        self.master.title("moive")
        self.master.geometry("400x300")
        
        self.canvas = tk.Canvas(self.master,width=400,height=300)
        self.canvas.pack(expand = True,fill=tk.BOTH)
        
        self.canvas.update()
        # self.canvas_w = self.canvas.winfo_width()
        # self.canvas_h = self.canvas.winfo_height()
        
        self.canvas_item = self.canvas.create_image(0,0,anchor=tk.NW)
        
    def disp_image(self,rgb_image):
        global tk_image
        bgr_image = cv2.cvtColor(rgb_image,cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(bgr_image)
        tk_image = ImageTk.PhotoImage(pil_image)
        
        self.canvas.itemconfig(self.canvas_item,image=tk_image)
        self.canvas.update()
        
            
        
        
if __name__ == '__main__':
    try:
        rclpy.init()
        image_converter = ImageConverter()
        
        thread_spin = threading.Thread(target=rclpy.spin,args=(image_converter,))
        #rclpy.spin(image_converter)
        thread_spin.start()
        image_converter.app.mainloop()

    except KeyboardInterrupt:
        thread_spin.join()
        cv2.destroyAllWindows()     
        image_converter.destroy_node
        rclpy.shutdown()   
