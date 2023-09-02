from UsbCan import UsbCan
import can
import time
Ucan = UsbCan()

def mainroop():
    for j in range(1,100):
        for i in range(1,100):
            msg = can.Message(
                arbitration_id=j,
                is_extended_id=False,
                data=[i,i,i,i,i,i,i,i]
            )
            Ucan.send(msg)
            print(msg)
            time.sleep(0.01)
            
    
Ucan.open()
mainroop()
Ucan.close()