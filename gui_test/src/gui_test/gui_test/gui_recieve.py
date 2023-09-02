import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class RecieveGui(Node):
    node_name = 'gui_sub'
    topic_name = 'chatter'
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.sub = self.create_subscription(String,self.topic_name,self.callback,10)
        self.setup_tk()
        self.label1["text"] = "setup done"
        print("setup done")
    
    def setup_tk(self):
        self.root = tk.Tk()
        self.root.title(u"subscriver")
        self.root.geometry("400x400")
        
        self.label1 = tk.Label()
        self.label1["text"] = "text"
        self.label1.pack()
        
        
    def callback(self,msg):
        print("{}".format(msg))
        self.label1["text"] = msg.data       
    
    
def main(args=None):
    rclpy.init(args=args)
    recv_gui = RecieveGui()
    thread_spin = threading.Thread(target=rclpy.spin,args=(recv_gui,))
    
    thread_spin.start()
    recv_gui.root.mainloop()
    thread_spin.join()
    recv_gui.root.destroy()
    recv_gui.destroy_node()
    rclpy.shutdown()
    # try:
    #     thread_spin.start()
    #     recv_gui.root.mainloop()
    # except KeyboardInterrupt:
    #     recv_gui.root.destroy()
    #     thread_spin.join()
    #     recv_gui.destroy_node()
    #     rclpy.shutdown()
        

if __name__ == '__main__':
    main()