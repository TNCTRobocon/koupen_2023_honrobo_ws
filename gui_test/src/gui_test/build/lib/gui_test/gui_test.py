import sys
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosGui(Node):
    node_name = "gui_pub"
    topic_name = "chatter"
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.pub = self.create_publisher(String,self.topic_name,10)
        self.msg = String()
    
    def send_entry(self,rcv_msg):
        print(str(rcv_msg))
        self.msg.data = str(rcv_msg)
        self.pub.publish(self.msg)
        
    

def main():
    rclpy.init(args=None)
    ros_gui = RosGui()
    
    root = tk.Tk()
    root.title(u"publisher")
    root.geometry("400x400")
    
    def quit():
        ros_gui.destroy_node()
        rclpy.shutdown()
        root.destroy()
    
    button1 = tk.Button()
    button1["text"] = "publish" 
    button1["command"] = lambda:ros_gui.send_entry(EditBox.get())
    button1.pack() 
    
    EditBox = tk.Entry(width=50)
    EditBox.insert(tk.END,"Hello World!")
    EditBox.pack()
    
    button2 = tk.Button()
    button2["text"] = "End" 
    button2["command"] = quit
    button2.pack() 

    root.mainloop()


if __name__ == '__main__':
    main()
