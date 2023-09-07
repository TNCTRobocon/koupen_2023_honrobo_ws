# OS
import time
import argparse

#ROS2-python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

#User-Module
from module.UsbCan import UsbCan 
from module.UseMessage import UseMessage
from module.Switch import SwitchStatus,ToggleSwitch



class JoyCan(Node):    
    NUM_OF_SAVE_STATE_BUTTONS = 2
    DEADZONE = 3
    PC = 1 # 0=Portable_PC 1=F310
    
    node_name = 'joy_can'
    topic_name = 'joy'
    
    def __init__(self,DEBUG) -> None:
        super().__init__(self.node_name)
        self.sub = self.create_subscription(Joy,self.topic_name,self.callback,10)
        self.Ucan = UsbCan(DEBUG)
        self.t_switch = ToggleSwitch(self.NUM_OF_SAVE_STATE_BUTTONS)
        self.Ucan.open()
        if self.NUM_OF_SAVE_STATE_BUTTONS:
            self.get_logger().info("setup done:DEBUG")
        else:
            self.get_logger().info("setup done")
    def __del__(self):
        self.Ucan.close()
        pass
        
        
    def recaluculating_joy(self,joy):
        recaluculated_joy = [0] * 8
        if self.PC == 0:
            ###Potable-PC
            recaluculated_joy[0] = joy.axes[0] * 127 + 128
            recaluculated_joy[1] = joy.axes[1] * 127 + 128
            recaluculated_joy[2] = joy.axes[3] * -127 + 128
            recaluculated_joy[3] = joy.axes[4] * -127 + 128
            recaluculated_joy[4] = joy.axes[2] * 127 + 128
            recaluculated_joy[5] = joy.axes[5] * 127 + 128
            recaluculated_joy[6] = 0
            recaluculated_joy[7] = 0
        elif self.PC == 1:
            ###F310
            recaluculated_joy[0] = joy.axes[0] * 127 + 128 #left-horizontal
            recaluculated_joy[1] = joy.axes[1] * 127 + 128 #left-vertical
            recaluculated_joy[2] = joy.axes[3] * 127 + 128 #right-horizontal
            recaluculated_joy[3] = joy.axes[4] * 127 + 128 #right-vertical
            recaluculated_joy[4] = joy.axes[2] * -127 + 128
            recaluculated_joy[5] = joy.axes[5] * -127 + 128
            recaluculated_joy[6] = 0 #none
            recaluculated_joy[7] = 0 #none
        else:
            pass
        
        ###common return
        recaluculated_joy = list(map(int,recaluculated_joy))
        return recaluculated_joy
    
    def recaluculating_hat(self,joy):
        recaluculated_hat = [0] * 8
        
        if self.PC == 0:
            ###Portable-PC
            recaluculated_hat[0] = joy.axes[6] + 1 #hat_vertical
            recaluculated_hat[0] = joy.axes[7] + 1 #hat_horizontal
        elif self.PC == 1:
            ###F310
            recaluculated_hat[0] = joy.axes[7] + 1 #hat_vertical
            recaluculated_hat[1] = joy.axes[6] + 1 #hat_horizontal
            recaluculated_hat[2] = 0
            recaluculated_hat[3] = 0
            recaluculated_hat[4] = 0
            recaluculated_hat[5] = 0
            recaluculated_hat[6] = 0
            recaluculated_hat[7] = 0
        else :
            pass    
        
        ###common return
        return recaluculated_hat
    
    def replase_button(self,joy,num):
        replased_button = [0] * num
        raw_button = [0] * 8
        for i in range(num):
            replased_button[i] = joy.buttons[i]
        for i in range(8):
            raw_button[i] = joy.buttons[i] 
        return raw_button,replased_button
    
    def copy_button(self,raw,pros):
        for i in range(len(pros)):
            raw[i] = pros[i]
        return raw
    
    def make_deadzone(self,data):
        for i in range(8):
            if data[i] >= self.DEADZONE:
                data[i] == 0
        return data
    
    def callback(self,joy):
        msg1 = UseMessage(1)
        msg2 = UseMessage(2)
        msg3 = UseMessage(3)
        status = SwitchStatus(1,3,self.NUM_OF_SAVE_STATE_BUTTONS)
        
        # CAN id1
        joy_data = [0] * 8
        joy_data = self.recaluculating_joy(joy)
        joy_msg_data = self.make_deadzone(joy_data)
        tx_msg1 = msg1.update_message(joy_msg_data)
        
        # CAN id2
        raw_button_data,toggle_button_data = self.replase_button(joy,self.NUM_OF_SAVE_STATE_BUTTONS)
        
        changed_switch = self.t_switch.judge_changed_button(toggle_button_data)
        if not changed_switch == -1:
            status.toggle_status(changed_switch)
        processed_button = status.get_status()

        copied_button = self.copy_button(raw_button_data,processed_button)
        tx_msg2 = msg2.update_message(copied_button)
        
        # CAN id3
        hat_msg_data = self.recaluculating_hat(joy)
        tx_msg3 = msg3.update_message(hat_msg_data)
        
        # Send CAN
        self.Ucan.send(tx_msg1)
        time.sleep(0.005)
        self.Ucan.send(tx_msg2)
        time.sleep(0.005)
        self.Ucan.send(tx_msg3)
        time.sleep(0.005)
        
        print_text = "axis:{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3}\nbutton_data:{},{},{},{},{},{},{},{}\nhat:{},{},{},{},{},{},{},{}"
        self.get_logger().info(print_text.format(*tx_msg1.data,*tx_msg2.data,*tx_msg3.data))
        
def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug" ,action="store_true" ,help="If --debug is added, CAN communication will not be performed.")
    cmd_arg = parser.parse_args()
    
    rclpy.init(args=args)
    joy_can = JoyCan(cmd_arg.debug)
    try:
        rclpy.spin(joy_can)
    except KeyboardInterrupt:
        pass
    finally:
        joy_can.destroy_node()

if __name__ == '__main__':
    main()
    