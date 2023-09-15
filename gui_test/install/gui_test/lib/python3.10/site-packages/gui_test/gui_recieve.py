import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RecieveGui(Node):
    node_name = "gui_sub"
    topic_name = "chatter"
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.sub = self.create_subscription(String,self.topic_name,self.callback,10)
        self.root = tk.Tk()
        self.root.title(u"subscriver")
        self.root.geometry("400x400")
        
        self.label1 = tk.Label()
        self.label1["text"] = ""
        self.label1.pack()
        
        self.button1 = tk.Button()
        self.button1["text"] = "End"
        self.button1["command"] = quit
        self.button1.pack()
        
    def callback(self,rcv_text):
        self.button1["text"] = str(rcv_text)
    
    def quit(self):
        self.destroy_node()
        rclpy.shutdown()
        self.root.destroy()
    
    
def main():
    rclpy.init(args=None)
    recv_gui = RecieveGui()

    recv_gui.root.mainloop()
    
if __name__ == '__main__':
    main()