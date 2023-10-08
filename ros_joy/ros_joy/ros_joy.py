# -*- coding: utf-8 -*-

# OS
import time
import struct

# ROS2-Python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

# User-Lib
from .module.Switch import SwitchStatus,ToggleSwitch
from .module.JoyCalcTools  import JoyCalcTools


class JoyCan(Node):    
    NUM_OF_SAVE_STATE_BUTTONS = 1
    DEADZONE = 3
    PC = 1 # 0=Portable_PC 1=F310
    
    node_name = 'ros_joy'
    sub_topic_name = 'converted_joy'
    joy_topic_name = 'can_joy'
    btn_topic_name = 'can_btn'
    data_topic_name = 'can_data'
    
    def __init__(self) -> None:
        super().__init__(self.node_name)
        self.sub = self.create_subscription(Joy,self.sub_topic_name,self.callback,10)
        
        self.pub_joy = self.create_publisher(Int16MultiArray,self.joy_topic_name,10)
        self.pub_btn = self.create_publisher(Int16MultiArray,self.btn_topic_name,10)
        self.pub_data = self.create_publisher(Int16MultiArray,self.data_topic_name,10)
        
        # self.Ucan = UsbCan(DEBUG)
        self.t_switch = ToggleSwitch(self.NUM_OF_SAVE_STATE_BUTTONS)
        self.tools = JoyCalcTools(self.PC)
        
        self.int_data = Int16MultiArray()
        
        # self.Ucan.open()
        
        self.get_logger().info("setup done")
    def __del__(self):
        self.Ucan.close()
        pass

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
        # msg1 = UseMessage(1)
        # msg2 = UseMessage(2)
        # msg3 = UseMessage(3)
        status = SwitchStatus(1,3,self.NUM_OF_SAVE_STATE_BUTTONS)
        
        
        # CAN id1
        joy_data = [0] * 8
        joy_data = self.tools.recaluculating_joy(joy)
        joy_msg_data = self.make_deadzone(joy_data)
        joy_msg_data = [int(i) for i in joy_msg_data]
        # tx_msg1 = msg1.update_message(joy_msg_data)
        tmp_data = Int16MultiArray(data=joy_msg_data)
        print(tmp_data)
        self.pub_joy.publish(tmp_data)

        
        
        # CAN id2
        raw_button_data,toggle_button_data = self.tools.replase_button(joy,self.NUM_OF_SAVE_STATE_BUTTONS)
        
        changed_switch = self.t_switch.judge_changed_button(toggle_button_data)
        if not changed_switch == -1:
            status.toggle_status(changed_switch)
        processed_button = status.get_status()

        copied_button = self.copy_button(raw_button_data,processed_button)
        copied_button = [int(i) for i in copied_button]
        tmp_data2 = Int16MultiArray(data=copied_button)

        print(tmp_data2)
        self.pub_btn.publish(tmp_data2)

        # tx_msg2 = msg2.update_message(copied_button)
        
        # CAN id3
        hat_msg_data = self.tools.recaluculating_hat(joy)
        hat_msg_data = [int(i) for i in hat_msg_data]
        tmp_data3 = Int16MultiArray(data=hat_msg_data)
        print(tmp_data3)
        self.pub_data.publish(tmp_data3)

        # tx_msg3 = msg3.update_message(hat_msg_data)
        
        # Send CAN
        # self.Ucan.send(tx_msg1)
        # time.sleep(0.005)
        # self.Ucan.send(tx_msg2)
        # time.sleep(0.005)
        # self.Ucan.send(tx_msg3)
        # time.sleep(0.005)
        
        print_text = "axis:{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3},{:0=3}\nbutton_data:{},{},{},{},{},{},{},{}\nhat:{},{},{},{},{},{},{},{}"
        #self.get_logger().info(print_text.format(*joy_msg_data,*copied_button,*hat_msg_data))

        
def main(args=None):
    
    rclpy.init(args=args)
    joy_can = JoyCan()
    try:
        rclpy.spin(joy_can)
    except KeyboardInterrupt:
        pass
    finally:
        joy_can.destroy_node()

if __name__ == '__main__':
    main()
    