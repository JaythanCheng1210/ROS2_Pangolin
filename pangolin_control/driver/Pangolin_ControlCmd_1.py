import os 
import time
import threading
import traceback

# if os.name == 'nt':
#     import msvcrt
#     def getch():
#         return msvcrt.getch().decode()
# else:
#     import sys, tty, termios
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     def getch():
#         try:
#             tty.setraw(sys.stdin.fileno())
#             ch = sys.stdin.read(1)
#         finally:
#             termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#         return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4         # Data Byte Length
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 1000      # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 2000      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0        

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
DXL4_ID                     = 4                 # Dynamixel#1 ID : 4
DXL5_ID                     = 5                 # Dynamixel#1 ID : 5


# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0

class Pangolincontrol_old():
    def __init__(self):
        self.X = 50
        self.servo_rate = [1.0, 1.0]
        self.dxl_goal_position_1 = [int((1673-self.X)*self.servo_rate[0]), int((1673-self.X)*self.servo_rate[0]), int((1173-self.X)*self.servo_rate[0]), int((1173-self.X)*self.servo_rate[0]), int((1473-self.X)*self.servo_rate[0]), int((1473-self.X)*self.servo_rate[0])]         # Goal position
        self.dxl_goal_position_2 = [int((2912+self.X)*self.servo_rate[1]), int((2622+self.X)*self.servo_rate[1]), int((2622+self.X)*self.servo_rate[1]), int((2422+self.X)*self.servo_rate[1]), int((2422+self.X)*self.servo_rate[1]), int((2912+self.X)*self.servo_rate[1])]         # Goal position
        self.dxl_goal_position_4 = [int((2572-self.X)*self.servo_rate[0]), int((2722-self.X)*self.servo_rate[0]), int((2722-self.X)*self.servo_rate[0]), int((2872-self.X)*self.servo_rate[0]), int((2872-self.X)*self.servo_rate[0]), int((2572-self.X)*self.servo_rate[0])]         # Goal position
        self.dxl_goal_position_5 = [int((1123+self.X)*self.servo_rate[1]), int((1123+self.X)*self.servo_rate[1]), int((1523+self.X)*self.servo_rate[1]), int((1523+self.X)*self.servo_rate[1]), int((1373+self.X)*self.servo_rate[1]), int((1373+self.X)*self.servo_rate[1])]  

        self.dxl_goal_position_3 = [1023, 1923]
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        self.init_fail = False
        self.is_walking = False
        self.is_curling = False
        self.walking_freq = 10
        self.openPort()
        self.setBaudrate()
        self.enableMotor()

        self.curl_index = 0

    def openPort(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            self.init_fail = True
    
    def setBaudrate(self):
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            self.init_fail = True
            
    def enableMotor(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL1_ID)

        # Enable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL2_ID)


        # Enable Dynamixel#4 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL4_ID)

        # Enable Dynamixel#5 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL5_ID)

        # Enable Dynamixel#3 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL3_ID)


        # self.packetHandler.write4ByteTxRx(self.portHandler, DXL3_ID, 112, 100)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL3_ID, 112, 50)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = self.groupSyncRead.addParam(DXL1_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)

        # Add parameter storage for Dynamixel#2 present position value
        dxl_addparam_result = self.groupSyncRead.addParam(DXL2_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)

        # Add parameter storage for Dynamixel#4 present position value
        dxl_addparam_result = self.groupSyncRead.addParam(DXL4_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL4_ID)

        # Add parameter storage for Dynamixel#5 present position value
        dxl_addparam_result = self.groupSyncRead.addParam(DXL5_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL5_ID)

        # Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = self.groupSyncRead.addParam(DXL3_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL3_ID)

    # def startCurl(self):
    #     index = 0
    #     dxl_goal_position = [1023, 1323]
    #     # Enable Dynamixel#3 Torque
    #     dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    #     else:
    #         print("Dynamixel#%d has been successfully connected" % DXL3_ID)

    #     # # Add parameter storage for Dynamixel#1 present position value
    #     # dxl_addparam_result = self.groupSyncRead.addParam(DXL3_ID)
    #     # if dxl_addparam_result != True:
    #     #     print("[ID:%03d] groupSyncRead addparam failed" % DXL3_ID)

    #     dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL3_ID, ADDR_GOAL_POSITION, dxl_goal_position[index])
    #     print(dxl_comm_result, dxl_error)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    #     if index == 0:
    #         index = 1
    #     else:
    #         index = 0

    def startCurl(self):

        param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position_3[self.curl_index])), DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position_3[self.curl_index])), 
                                DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position_3[self.curl_index])), DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position_3[self.curl_index]))]

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL3_ID, param_goal_position_3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        if self.curl_index == 0:
            self.curl_index = 1
        else:
            self.curl_index = 0

        print(self.curl_index)

    def stopCurl(self):
        pass

    def processGait(self):
        index = 0
        index_left  = 0
        index_right = 0 
        while self.is_walking:

            # self.dxl_goal_position_1 = [int((1673-self.X)*self.servo_rate[0]), int((1673-self.X)*self.servo_rate[0]), int((1173-self.X)*self.servo_rate[0]), int((1173-self.X)*self.servo_rate[0]), int((1473-self.X)*self.servo_rate[0]), int((1473-self.X)*self.servo_rate[0])]         # Goal position
            # self.dxl_goal_position_2 = [int((2912+self.X)*self.servo_rate[1]), int((2622+self.X)*self.servo_rate[1]), int((2622+self.X)*self.servo_rate[1]), int((2422+self.X)*self.servo_rate[1]), int((2422+self.X)*self.servo_rate[1]), int((2912+self.X)*self.servo_rate[1])]         # Goal position
            # self.dxl_goal_position_4 = [int((2572-self.X)*self.servo_rate[0]), int((2722-self.X)*self.servo_rate[0]), int((2722-self.X)*self.servo_rate[0]), int((2872-self.X)*self.servo_rate[0]), int((2872-self.X)*self.servo_rate[0]), int((2572-self.X)*self.servo_rate[0])]         # Goal position
            # self.dxl_goal_position_5 = [int((1123+self.X)*self.servo_rate[1]), int((1123+self.X)*self.servo_rate[1]), int((1523+self.X)*self.servo_rate[1]), int((1523+self.X)*self.servo_rate[1]), int((1373+self.X)*self.servo_rate[1]), int((1373+self.X)*self.servo_rate[1])]  


            param_goal_position_1 = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position_1[index_left])),  DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position_1[index_left])),   DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position_1[index_left])),   DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position_1[index_left]))    ]
            param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position_2[index_right])), DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position_2[index_right])),  DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position_2[index_right])),  DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position_2[index_right]))   ]
            param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position_4[index_left])),  DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position_4[index_left])),   DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position_4[index_left])),   DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position_4[index_left]))    ]
            param_goal_position_5 = [DXL_LOBYTE(DXL_LOWORD(self.dxl_goal_position_5[index_right])), DXL_HIBYTE(DXL_LOWORD(self.dxl_goal_position_5[index_right])),  DXL_LOBYTE(DXL_HIWORD(self.dxl_goal_position_5[index_right])),  DXL_HIBYTE(DXL_HIWORD(self.dxl_goal_position_5[index_right]))   ]

            # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(DXL1_ID, param_goal_position_1)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)

            # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(DXL2_ID, param_goal_position_2)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)

            # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(DXL4_ID, param_goal_position_4)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)

            # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(DXL5_ID, param_goal_position_5)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)


            # Syncwrite goal position
            dxl_comm_result = self.groupSyncWrite.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Clear syncwrite parameter storage
            self.groupSyncWrite.clearParam()

            # print("index: %d" % index)
            if self.servo_rate[0] > 0:
                index_left += 1
                index_right += 1

                if index_left >= len(self.dxl_goal_position_1):
                    index_left = 0
                    index_right = 0

            elif self.servo_rate[0] < 0:
                index_left -= 1
                index_right -= 1

                if index_left < 0:
                    index_left = len(self.dxl_goal_position_1)-1
                    index_right = len(self.dxl_goal_position_1)-1
            else:
                if self.servo_rate[1] > 0:
                    index_left += 1
                    index_right -= 1

                    if index_left >= len(self.dxl_goal_position_1):
                        index_left = 0
                        index_right = len(self.dxl_goal_position_1)-1
                
                elif self.servo_rate[1] < 0:
                    index_left -= 1
                    index_right += 1

                    if index_left < 0:
                        index_left = len(self.dxl_goal_position_1)-1
                        index_right = 0

            
            time.sleep(1 / self.walking_freq)

    def startGait(self):
        if not self.init_fail:
            self.is_walking = True
            self.walking_thread = threading.Thread(target=self.processGait, daemon=True)
            self.walking_thread.start()
        else:
            print("Error Occurr")
            
    def set_servo_rate(self, servo_rate):
        self.servo_rate = servo_rate

    def setWalkingFreq(self, freq):
        self.walking_freq = freq
        
    def stopWalking(self):
        self.is_walking = False

            
    def disableMotor(self):
        self.stopWalking()
        # Clear syncread parameter storage
        self.groupSyncRead.clearParam()
        
        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL5_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()
        
def main():
    controller = Pangolincontrol_old()
    cmd_dict = {
        "w": controller.startGait,
        "s": controller.stopWalking,
        "c": controller.startCurl,
    }
    
    while True:
        try:
            cmd = input("CMD: ")
            if cmd in cmd_dict:
                if cmd == "w":
                    freq = input("Freq: ")
                    controller.setWalkingFreq(float(freq))
                cmd_dict[cmd]()
            elif cmd == "exit":
                controller.disableMotor()
                break
        except Exception as e:
            traceback.print_exc()
            break

if __name__ == "__main__":
    main()