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
    statu = [1, 1, 1, 1, 1, 1]
    config = [3, 2, 2, 2, 2, 2]
    
    color_config = ["#3a7ebf", "#bf3a7a", "#3abf60"]
    color_hover_config = ["#325882", "#823275", "#32823f"]
    text_config_mat = []
    
    text_config_1 = ["今:通過固定モード", "今:通過前進同期モード", "今:通過初期位置復帰"]
    text_config_mat.append(text_config_1)
    text_config_2 = ["今:ベルト固定モード", "今:ベルト動くモード"]
    text_config_mat.append(text_config_2)
    text_config_3 = ["今:前輪DOWN", "今:前輪UP"]
    text_config_mat.append(text_config_3)
    text_config_4 = ["今:補助輪UP", "今:補助輪DOWN"]
    text_config_mat.append(text_config_4)
    text_config_5 = ["今:後輪DOWN", "今:後輪UP"]
    text_config_mat.append(text_config_5)
    text_config_6 = ["今:自動操縦ON", "今:自動操縦OFF"]
    text_config_mat.append(text_config_6)
    
    preset_config = [[1, 1, 1], [1, 2, 1], [2, 2, 1], [2, 1, 1], [1, 1, 1], [1, 1, 2], [1, 2, 2], [1, 2, 1], [1, 1, 1]]
    preset = 0
    
    button_mat = []
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.ros_gui = RosGui()
        
        self.fonts = (self.FONT_TYPE, 15)
        self.geometry("450x800")
        self.title("GUI")
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=1)
        self.grid_rowconfigure(3, weight=1)
        self.grid_rowconfigure(4, weight=1)
        self.grid_rowconfigure(5, weight=1)
        self.grid_rowconfigure(6, weight=1)
        self.grid_rowconfigure(7, weight=1)
        
        self.setup_form()
        
    
    def check_config(self,num):
        if self.statu[num]  > self.config[num]:
            self.statu[num] = 1
            
    def change_step(self,mode):
        if mode: #increse
            self.preset += 1
            if self.preset > 7:
                self.preset = 0
        else:   #decrese
            self.preset -= 1
            if self.preset < 0:
                self.preset = 7
        
        print("preset :" + str(self.preset))
        for i in range(3):
            self.statu[i + 2] = self.preset_config[self.preset][i]
            
    def override_gui(self):
        for i in range(6):
            self.button_mat[i].configure(fg_color=self.color_config[self.statu[i] - 1])
            self.button_mat[i].configure(text=self.text_config_mat[i][self.statu[i] - 1])
            self.button_mat[i].configure(hover_color=self.color_hover_config[self.statu[i] - 1])

        self.ros_gui.cvt_and_send(self.statu)
        print(self.statu)
            
    def setup_form(self):
        ct.set_appearance_mode("dark")
        ct.set_default_color_theme("blue")
        
        self.button1 = ct.CTkButton(master=self, width=200, height=100, text=self.text_config_1[0], command=self.callback, font=self.fonts)
        self.button1.grid(column=0, row=0, padx=5, pady=5)# , sticky="ew"
        self.button_mat.append(self.button1)
        
        self.button2 = ct.CTkButton(master=self, width=200, height=100, text=self.text_config_2[0], command=self.callback2, font=self.fonts)
        self.button2.grid(column=0, row=1, padx=5, pady=5)
        self.button_mat.append(self.button2)
        
        self.button3 = ct.CTkButton(master=self, width=200, height=100, text=self.text_config_3[0], command=self.callback3, font=self.fonts)
        self.button3.grid(column=0, row=2, padx=5, pady=5)
        self.button_mat.append(self.button3)
        
        self.button3_2 = ct.CTkButton(master=self, width=200, height=100, text=self.text_config_3[0], command=self.callback3, font=self.fonts)
        self.button3_2.grid(column=1, row=2, padx=5, pady=5)
        # self.button_mat.append(self.button3)
        
        self.button4 = ct.CTkButton(master=self, width=200, height=100, text=self.text_config_4[0], command=self.callback4, font=self.fonts)
        self.button4.grid(column=0, row=3, padx=5, pady=5)
        self.button_mat.append(self.button4)
        
        self.button5 = ct.CTkButton(master=self, width=200, height=100, text=self.text_config_5[0], command=self.callback5, font=self.fonts)
        self.button5.grid(column=0, row=4, padx=5, pady=5)
        self.button_mat.append(self.button5)
               
        self.button6 = ct.CTkButton(master=self, width=200, height=100, text=self.text_config_6[0], command=self.callback6, font=self.fonts)
        self.button6.grid(column=0, row=5, padx=5, pady=5)
        self.button_mat.append(self.button6)
                
        self.button7 = ct.CTkButton(master=self, width=200, height=100, text="ロープ通過:ステップを進む", command=self.callback7, font=self.fonts)
        self.button7.grid(column=0, row=6, padx=5, pady=5)
        
        self.button8 = ct.CTkButton(master=self, width=200, height=100, text="ロープ通過:ステップを戻る", command=self.callback8, font=self.fonts)
        self.button8.grid(column=0, row=7, padx=5, pady=5)
        

        
    def callback(self):
        self.statu[0] += 1 
        self.check_config(0)
        self.override_gui()
        

    def callback2(self):
        self.statu[1] += 1 
        self.check_config(1)
        self.override_gui()
        
    def callback3(self):
        self.statu[2] += 1 
        self.check_config(2)
        self.override_gui()
        
    def callback4(self):
        self.statu[3] += 1 
        self.check_config(3)
        self.override_gui()
        
    def callback5(self):
        self.statu[4] += 1 
        self.check_config(4)
        self.override_gui()

    def callback6(self):
        self.statu[5] += 1 
        self.check_config(5)
        self.override_gui()
        
    def callback7(self):
        self.change_step(1)
        self.override_gui()
        
    
    def callback8(self):
        self.change_step(0)
        self.override_gui()
    
    
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