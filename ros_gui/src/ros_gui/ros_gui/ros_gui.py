import customtkinter as ct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

FONT_TYPE = "meiryo"


class App(ct.CTk):
    status = [0,0,0,0,0,0]
    color_config = ["#3a7ebf","#bf3a7a"]
    color_hover_config = ["#325882","#823275"]
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.ros_gui = RosGui()
        
        self.fonts = (FONT_TYPE, 15)
        self.geometry("350x500")
        self.title("GUI")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)
        self.grid_rowconfigure(5, weight=1)
        
        self.setup_form()


    def setup_form(self):
        ct.set_appearance_mode("dark")
        ct.set_default_color_theme("blue")
        
        self.button1 = ct.CTkButton(master=self, width=200, height=80, text="Button1", command=self.callback, font=self.fonts)
        self.button1.grid(column=0, row=0, padx=5, pady=5, sticky="ew")
        
        self.button2 = ct.CTkButton(master=self, width=200, height=80, text="Button2", command=self.callback2, font=self.fonts)
        self.button2.grid(column=0, row=1, padx=5, pady=5, sticky="ew")
        
        self.button3 = ct.CTkButton(master=self, width=200, height=80, text="Button3", command=self.callback3, font=self.fonts)
        self.button3.grid(column=0, row=2, padx=5, pady=5, sticky="ew")

        self.button4 = ct.CTkButton(master=self, width=200, height=80, text="Button4", command=self.callback4, font=self.fonts)
        self.button4.grid(column=0, row=3, padx=5, pady=5, sticky="ew")

        self.button5 = ct.CTkButton(master=self, width=200, height=80, text="Button5", command=self.callback5, font=self.fonts)
        self.button5.grid(column=0, row=4, padx=5, pady=5, sticky="ew")
        
        self.button6 = ct.CTkButton(master=self, width=200, height=80, text="Button6", command=self.callback6, font=self.fonts)
        self.button6.grid(column=0, row=5, padx=5, pady=5, sticky="ew")
        
    def callback(self):
        self.status[0] = ~self.status[0] & 1
        self.button1.configure(fg_color=self.color_config[self.status[0]])
        self.button1.configure(hover_color=self.color_hover_config[self.status[0]])
        self.ros_gui.cvt_and_send(self.status)

    def callback2(self):
        self.status[1] = ~self.status[1] & 1
        self.button2.configure(fg_color=self.color_config[self.status[1]])
        self.button2.configure(hover_color=self.color_hover_config[self.status[1]])
        self.ros_gui.cvt_and_send(self.status)
        
    def callback3(self):
        self.status[2] = ~self.status[2] & 1
        self.button3.configure(fg_color=self.color_config[self.status[2]])
        self.button3.configure(hover_color=self.color_hover_config[self.status[2]])
        self.ros_gui.cvt_and_send(self.status)
        
    def callback4(self):
        self.status[3] = ~self.status[3] & 1
        self.button4.configure(fg_color=self.color_config[self.status[3]])
        self.button4.configure(hover_color=self.color_hover_config[self.status[3]])
        self.ros_gui.cvt_and_send(self.status)
        
    def callback5(self):
        self.status[4] = ~self.status[4] & 1
        self.button5.configure(fg_color=self.color_config[self.status[4]])
        self.button5.configure(hover_color=self.color_hover_config[self.status[4]])
        self.ros_gui.cvt_and_send(self.status)

    def callback6(self):
        self.status[5] = ~self.status[5] & 1
        self.button6.configure(fg_color=self.color_config[self.status[5]])
        self.button6.configure(hover_color=self.color_hover_config[self.status[5]])
        self.ros_gui.cvt_and_send(self.status)
        

class RosGui(Node):
    node_name = "ros_gui"
    config_sub_topic_name = "config"
    
    def __init__(self):
        super().__init__(self.node_name)
        self.pub_config = self.create_publisher(Int16MultiArray,self.config_sub_topic_name,10)

    def cvt_and_send(self,data):
        send_data = Int16MultiArray(data=data)
        self.pub_config.publish(send_data)
        print(data)
        
def main():
    try: 
        app = App()
        app.mainloop()
    except KeyboardInterrupt:
        app.destroy()
        
if __name__ == '__main__':
    main()