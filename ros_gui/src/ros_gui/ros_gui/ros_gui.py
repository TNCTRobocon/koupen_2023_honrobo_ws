import customtkinter as ct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

    

class App(ct.CTk):
    FONT_TYPE = "meiryo"
    config_keeper = [1, 1, 1, 1, 1, 1]
    num_of_config = [3, 2, 2, 2, 2, 2]
    
    color_config = ["#bf3a7a", "#3a7ebf"]
    color_hover_config = ["#823275", "#325882"]
    
    button_obj_keeper = []
    now_preset = 0
    preset_config = [[1, 1, 1], [1, 2, 1], [2, 2, 1], [2, 1, 1], [1, 1, 1], [1, 1, 2], [1, 2, 2], [1, 2, 1], [1, 1, 1]]
    
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.ros_gui = RosGui()
        
        self.fonts = (self.FONT_TYPE, 25)
        self.geometry("590x900")
        self.title("GUI")
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)
        self.grid_rowconfigure(5, weight=1)
        self.grid_rowconfigure(6, weight=1)
        self.grid_rowconfigure(7, weight=1)
        
        self.setup_form()
        
            
    def setup_form(self):
        ct.set_appearance_mode("dark")
        ct.set_default_color_theme("blue")
        
        self.conf1_btn1 = ct.CTkButton(master=self, width=180, height=100, text="通過しない", command=lambda a = 1, b = 1, c = 1 :self.callback(a,b,c), font=self.fonts)
        self.conf1_btn1.grid(column=0, row=5, padx=5, pady=5)
        self.conf1_btn2 = ct.CTkButton(master=self, width=180, height=100, text="通過する", command=lambda a = 1, b = 1, c = 2 :self.callback(a,b,c), font=self.fonts)
        self.conf1_btn2.grid(column=1, row=5, padx=5, pady=5)
        self.conf1_btn3 = ct.CTkButton(master=self, width=180, height=100, text="はじめに戻す", command=lambda a = 1, b = 1, c = 3 :self.callback(a,b,c), font=self.fonts)
        self.conf1_btn3.grid(column=2, row=5, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf1_btn1, self.conf1_btn2, self.conf1_btn3])
        
        self.conf2_btn1 = ct.CTkButton(master=self, width=180, height=80, text="うごかない", command=lambda a = 2, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf2_btn1.grid(column=0, row=0, padx=5, pady=5)
        self.conf2_label = ct.CTkLabel(master=self, width=180, height=80, text="ベルト", font=self.fonts)
        self.conf2_label.grid(column=1, row=0, padx=5, pady=5)
        self.conf2_btn2 = ct.CTkButton(master=self, width=180, height=80, text="うごく", command=lambda a = 2, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf2_btn2.grid(column=2, row=0, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf2_btn1, self.conf2_btn2])
        
        self.conf3_btn1 = ct.CTkButton(master=self, width=180, height=80, text="さがる", command=lambda a = 3, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf3_btn1.grid(column=0, row=1, padx=5, pady=5)
        self.conf3_label = ct.CTkLabel(master=self, width=180, height=80, text="前タイヤ", font=self.fonts)
        self.conf3_label.grid(column=1, row=1, padx=5, pady=5)
        self.conf3_btn2 = ct.CTkButton(master=self, width=180, height=80, text="あがる", command=lambda a = 3, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf3_btn2.grid(column=2, row=1, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf3_btn1, self.conf3_btn2])
        
        self.conf4_btn1 = ct.CTkButton(master=self, width=180, height=80, text="あがる", command=lambda a = 4, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf4_btn1.grid(column=0, row=2, padx=5, pady=5)
        self.conf4_label = ct.CTkLabel(master=self, width=180, height=80, text="補助輪", font=self.fonts)
        self.conf4_label.grid(column=1, row=2, padx=5, pady=5)
        self.conf4_btn2 = ct.CTkButton(master=self, width=180, height=80, text="さがる", command=lambda a = 4, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf4_btn2.grid(column=2, row=2, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf4_btn1, self.conf4_btn2])
        
        self.conf5_btn1 = ct.CTkButton(master=self, width=180, height=80, text="さがる", command=lambda a = 5, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf5_btn1.grid(column=0, row=3, padx=5, pady=5)
        self.conf5_label = ct.CTkLabel(master=self, width=180, height=80, text="後ろタイヤ", font=self.fonts)
        self.conf5_label.grid(column=1, row=3, padx=5, pady=5)
        self.conf5_btn2 = ct.CTkButton(master=self, width=180, height=80, text="あがる", command=lambda a = 5, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf5_btn2.grid(column=2, row=3, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf5_btn1, self.conf5_btn2])
        
        self.conf6_btn1 = ct.CTkButton(master=self, width=180, height=80, text="オン", command=lambda a = 6, b = 1, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf6_btn1.grid(column=0, row=4, padx=5, pady=5)
        self.conf6_label = ct.CTkLabel(master=self, width=180, height=80, text="自動操縦", font=self.fonts)
        self.conf6_label.grid(column=1, row=4, padx=5, pady=5)
        self.conf6_btn2 = ct.CTkButton(master=self, width=180, height=80, text="オフ", command=lambda a = 6, b = 0, c = 0 :self.callback(a,b,c), font=self.fonts)
        self.conf6_btn2.grid(column=2, row=4, padx=5, pady=5)
        
        self.button_obj_keeper.append([self.conf6_btn1, self.conf6_btn2])
        
        self.conf7_btn1 = ct.CTkButton(master=self, width=180, height=200, text="戻る", command=lambda a = 0:self.apply_preset(a), font=self.fonts)
        self.conf7_btn1.grid(column=0, row=6, padx=5, pady=5)
        self.conf7_label = ct.CTkLabel(master=self, width=180, height=200, text="段差乗り越え %d/%d"%(self.now_preset,len(self.preset_config) - 1), font=self.fonts)
        self.conf7_label.grid(column=1, row=6, padx=5, pady=5)
        self.conf7_btn2 = ct.CTkButton(master=self, width=180, height=200, text="進む", command=lambda a = 1:self.apply_preset(a), font=self.fonts)
        self.conf7_btn2.grid(column=2, row=6, padx=5, pady=5)

        self.updates()
    
    def callback(self,config,mode,me):
        if config == 1:
            self.config_keeper[config - 1] = me
        else:
            if mode == 1:
                self.config_keeper[config - 1] += 1
                if self.config_keeper[config - 1] >= self.num_of_config[config - 1]:
                    self.config_keeper[config - 1] = 1
            elif mode == 0:
                self.config_keeper[config - 1] -= 1
                if self.config_keeper[config - 1] <= 1:
                    self.config_keeper[config - 1] = self.num_of_config[config - 1]
        self.updates()
            
    def updates(self):
        print(self.config_keeper)
        for i in range(len(self.button_obj_keeper)):
            i_list = self.button_obj_keeper[i]
            for j in range(len(i_list)):
                target_obj = i_list[j] 
                target_obj.configure(fg_color=self.color_config[j + 1 != self.config_keeper[i]])
                target_obj.configure(hover_color=self.color_config[j + 1 != self.config_keeper[i]])
        
        self.ros_gui.cvt_and_send(self.config_keeper)
        
    def apply_preset(self,mode):
        if mode: # Next
            self.now_preset += 1
            if self.now_preset > len(self.preset_config) - 1:
                self.now_preset = 0
        else:
            self.now_preset -= 1
            if self.now_preset < 0:
                self.now_preset = len(self.preset_config) - 1
                
        for i in range(3):
            self.config_keeper[i + 2] = self.preset_config[self.now_preset][i]
        
        if self.now_preset == 0:
            self.config_keeper[1] = 1
        else:
            self.config_keeper[1] = 2
            
        self.conf7_label.configure(text="段差乗り越え %d/%d"%(self.now_preset,len(self.preset_config) - 1))
        self.updates()
    
class RosGui(Node):
    node_name = "ros_gui"
    config_sub_topic_name = "config"
    
    def __init__(self):
        super().__init__(self.node_name)
        self.pub_config = self.create_publisher(Int16MultiArray, self.config_sub_topic_name, 10)

    def cvt_and_send(self,data):
        send_data = Int16MultiArray(data=data)
        self.pub_config.publish(send_data)
        
def main():
    try: 
        app = App()
        app.mainloop()
    except KeyboardInterrupt:
        app.destroy()
        
if __name__ == '__main__':
    main()