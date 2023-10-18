import customtkinter as ct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray



class Switch():
    def __init__(self,num):
        self.num = num
        self.keep = 1
        
    def countup(self):
        self.keep += 1
        if self.keep > self.num:
            self.keep = 1
            
    def countdown(self):
        self.keep -= 1
        if self.keep < 1:
            self.keep = self.num
            
    def return_index(self):
        return self.keep - 1
    

class App(ct.CTk):
    FONT_TYPE = "meiryo"
    color_config = ["#3a7ebf", "#bf3a7a", "#3abf60"]
    color_hover_config = ["#325882", "#823275", "#32823f"]
    text_config_1 = ["今:通過動かないモード", "今:通過前進同期モード", "今:通過初期位置モード"]
    text_config_2 = ["今:ベルト動かないモード", "今:ベルト動くモード"]
    text_config_3 = ["今:前輪DOWN", "今:前輪UP"]
    text_config_4 = ["今:補助輪UP", "今:補助輪DOWN"]
    text_config_5 = ["今:後輪DOWN", "今:後輪UP"]
    text_config_6 = ["今:おまけON", "今:おまけOFF"]
    
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.ros_gui = RosGui()
        
        self.fonts = (self.FONT_TYPE, 15)
        self.geometry("350x500")
        self.title("GUI")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)
        self.grid_rowconfigure(5, weight=1)
        
        self.btn1 = Switch(3)
        self.btn2 = Switch(2)
        self.btn3 = Switch(2)
        self.btn4 = Switch(2)
        self.btn5 = Switch(2)
        self.btn6 = Switch(2)
        
        self.setup_form()


    def setup_form(self):
        ct.set_appearance_mode("dark")
        ct.set_default_color_theme("blue")
        
        self.button1 = ct.CTkButton(master=self, width=200, height=80, text=self.text_config_1[0], command=self.callback, font=self.fonts)
        self.button1.grid(column=0, row=0, padx=5, pady=5, sticky="ew")
        
        self.button2 = ct.CTkButton(master=self, width=200, height=80, text=self.text_config_2[0], command=self.callback2, font=self.fonts)
        self.button2.grid(column=0, row=1, padx=5, pady=5, sticky="ew")
        
        self.button3 = ct.CTkButton(master=self, width=200, height=80, text=self.text_config_3[0], command=self.callback3, font=self.fonts)
        self.button3.grid(column=0, row=2, padx=5, pady=5, sticky="ew")

        self.button4 = ct.CTkButton(master=self, width=200, height=80, text=self.text_config_4[0], command=self.callback4, font=self.fonts)
        self.button4.grid(column=0, row=3, padx=5, pady=5, sticky="ew")

        self.button5 = ct.CTkButton(master=self, width=200, height=80, text=self.text_config_5[0], command=self.callback5, font=self.fonts)
        self.button5.grid(column=0, row=4, padx=5, pady=5, sticky="ew")
        
        self.button6 = ct.CTkButton(master=self, width=200, height=80, text=self.text_config_6[0], command=self.callback6, font=self.fonts)
        self.button6.grid(column=0, row=5, padx=5, pady=5, sticky="ew")
        
    def callback(self):
        self.btn1.countup()
        self.button1.configure(fg_color=self.color_config[self.btn1.return_index()])
        self.button1.configure(hover_color=self.color_hover_config[self.btn1.return_index()])
        self.button1.configure(text=self.text_config_1[self.btn1.return_index()])
        self.status[0] = self.btn1.keep
        self.ros_gui.cvt_and_send(self.status)

    def callback2(self):
        self.btn2.countup()
        self.button2.configure(fg_color=self.color_config[self.btn2.return_index()])
        self.button2.configure(hover_color=self.color_hover_config[self.btn2.return_index()])
        self.button2.configure(text=self.text_config_2[self.btn2.return_index()])
        self.status[1] = self.btn2.keep
        self.ros_gui.cvt_and_send(self.status)
        
    def callback3(self):
        self.btn3.countup()
        self.button3.configure(fg_color=self.color_config[self.btn3.return_index()])
        self.button3.configure(hover_color=self.color_hover_config[self.btn3.return_index()])
        self.button3.configure(text=self.text_config_3[self.btn3.return_index()])
        self.status[2] = self.btn3.keep
        self.ros_gui.cvt_and_send(self.status)
        
    def callback4(self):
        self.btn4.countup()
        self.button4.configure(fg_color=self.color_config[self.btn4.return_index()])
        self.button4.configure(hover_color=self.color_hover_config[self.btn4.return_index()])
        self.button4.configure(text=self.text_config_4[self.btn4.return_index()])
        self.status[3] = self.btn4.keep
        self.ros_gui.cvt_and_send(self.status)
        
    def callback5(self):
        self.btn5.countup()
        self.button5.configure(fg_color=self.color_config[self.btn5.return_index()])
        self.button5.configure(hover_color=self.color_hover_config[self.btn5.return_index()])
        self.button5.configure(text=self.text_config_5[self.btn5.return_index()])
        self.status[4] = self.btn5.keep
        self.ros_gui.cvt_and_send(self.status)

    def callback6(self):
        self.btn6.countup()
        self.button6.configure(fg_color=self.color_config[self.btn6.return_index()])
        self.button6.configure(hover_color=self.color_hover_config[self.btn6.return_index()])
        self.button6.configure(text=self.text_config_6[self.btn6.return_index()])
        self.status[5] = self.btn6.keep
        self.ros_gui.cvt_and_send(self.status)
        

class RosGui(Node):
    node_name = "ros_gui"
    config_sub_topic_name = "config"
    
    def __init__(self):
        super().__init__(self.node_name)
        self.pub_config = self.create_publisher(Int16MultiArray, self.config_sub_topic_name,10)

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