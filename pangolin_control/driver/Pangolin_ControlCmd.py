from DXL_motor_control import DXL_Conmunication
import time
import sys, tty, termios
import traceback
import json 
import threading
import log
from time import sleep
from Board import setPWMServoPulse
from Pangolin_ActionGroups import action_dic
from Pangolin_Config import *
from Pangolin_Stance import PangolinStance


class PangolinControl:
    def __init__(self):
        self.control_cmd = ControlCmd()
        self.stance_cmd = PangolinStance()
        self.motor_center_position = {'motor1': 1535, 'motor2': 2561, 'motor3': 992, 'motor4': 2567, 'motor5': 1525}
        setPWMServoPulse(5, 1500, 100)
        setPWMServoPulse(6, 1500, 100)
        
        self.init_fail = False
        self.is_walking = False
        self.is_turning = False
        
        self.x = 0
        self.z = LEG_HEIGHT
        self.pitch = 0
        self.roll = 0

        self.servo_rate = [1.0, 1.0]

        self.is_led_blink = True
        self.is_recording = False
        self.gait_name = 'move_linear'
        self.record_path = '/home/ubuntu/pangolin_ws/ros2-pangolin-robot/pangolin_control/driver/output.txt'

        self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(0, 'motor1'), 
                                                                "motor2":self.inverse_kinematic(0, 'motor2'), 
                                                                "motor3":self.inverse_kinematic(0, 'motor3'),
                                                                "motor4":self.inverse_kinematic(0, 'motor4'),
                                                                "motor5":self.inverse_kinematic(0, 'motor5')})


    # Reset to the original state
    def reset_to_orginal(self):  
        setPWMServoPulse(5, 1500, 100)
        setPWMServoPulse(6, 1500, 100)  
        self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(0, 'motor1'), 
                                                                "motor2":self.inverse_kinematic(0, 'motor2'), 
                                                                "motor3":self.inverse_kinematic(0, 'motor3'),
                                                                "motor4":self.inverse_kinematic(0, 'motor4'),
                                                                "motor5":self.inverse_kinematic(0, 'motor5')})


    def inverse_kinematic(self, leg_pos, motor_name):
        leg_side = 0
        direction = 1
        if motor_name == 'motor1':
            leg_side = 0
            direction = 1
        elif motor_name == 'motor2':
            leg_side = 1
            direction = -1
        elif motor_name == 'motor4':
            leg_side = 0
            direction = 1
        elif motor_name == 'motor5':
            leg_side = 1
            direction = -1
        else:
            return int(self.motor_center_position[motor_name])
        
        return int(direction*leg_pos* self.servo_rate[leg_side] + self.motor_center_position[motor_name])
    
    # Pangolin move gait process
    def process_gait(self):
        if self.gait_name == 'move_linear':
            while True:
                if self.is_walking == False: break
                setPWMServoPulse(5, 1700, 100)
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic( leg_forward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(leg_backward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(leg_backward, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic( leg_forward, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic( leg_forward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(           0, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(           0, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic( leg_forward, 'motor5')})

                if self.is_walking == False: break
                setPWMServoPulse(5, 1500, 100)
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(leg_backward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(           0, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(           0, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(leg_backward, 'motor5')})
                

                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(leg_backward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic( leg_forward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic( leg_forward, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(leg_backward, 'motor5')})
                
                if self.is_walking == False: break
                setPWMServoPulse(5, 1300, 100)
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(           0, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic( leg_forward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic( leg_forward, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(           0, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(           0, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(leg_backward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(leg_backward, 'motor4'),
                                                                        "motor5":self.inverse_kinematic(           0, 'motor5')})
                setPWMServoPulse(5, 1500, 100)

                

        elif self.gait_name == 'turn_right':
            while True:
                if self.is_walking == False: break
                setPWMServoPulse(5, 1300, 100)
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(           0, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(turn_backward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'),
                                                                        "motor4":self.inverse_kinematic(turn_forward, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(           0, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(turn_backward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(turn_backward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(turn_forward, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(turn_forward, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(turn_backward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(           0, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(           0, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(turn_forward, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(           0, 'motor1'),
                                                                        "motor2":self.inverse_kinematic(           0, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(           0, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(           0, 'motor5')})

        elif self.gait_name == 'turn_left':
            while True:
                if self.is_walking == False: break
                setPWMServoPulse(5, 1700, 100)
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(           0, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(           0, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(           0, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(           0, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(turn_backward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(           0, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(           0, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(turn_forward, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(turn_backward, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(turn_backward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(turn_forward, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(turn_forward, 'motor5')})
                
                if self.is_walking == False: break
                self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(           0, 'motor1'), 
                                                                        "motor2":self.inverse_kinematic(turn_backward, 'motor2'), 
                                                                        "motor3":self.inverse_kinematic(           0, 'motor3'), 
                                                                        "motor4":self.inverse_kinematic(turn_forward, 'motor4'), 
                                                                        "motor5":self.inverse_kinematic(           0, 'motor5')})

    # Start moving 
    def start_gait(self):
        self.is_walking = True
        self.is_turning = True

        self.walking_thread = threading.Thread(target=self.process_gait, args=(), daemon=True)
        self.walking_thread.start()

    # Stop moving 
    def stop_gait(self):
        self.is_walking = False
        self.is_turning = False

        self.reset_to_orginal()

    # Set the twist msg to left and right side of the motors
    def set_servo_rate(self, servo_rate=[1.0, 1.0]):
        self.servo_rate = servo_rate

    # Set the twist msg to left and right side of the motors
    def set_gait_name(self, gait_name='move_linear'):
        self.gait_name = gait_name

    # The process of the recording function
    def process_record_action_points(self):
        self.control_cmd.disable_all_motor()
        print("disabling")
        self.control_cmd.dynamixel.rebootAllMotor()
        print("rebooting")
        self.control_cmd.motor_led_control(LED_ON)
        
        with open(self.record_path, 'w') as f:
            print("start record the action points....")
            while self.is_recording:
                all_servo_position = self.control_cmd.read_all_motor_data()
                print(f"recording: {all_servo_position}")
                f.write(json.dumps(all_servo_position)+'\n')
                time.sleep(0.01)
            self.control_cmd.motor_led_control(LED_OFF)
        print("finish recording!")

    # Start recording the motors position
    def start_record_action_points(self):
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self.process_record_action_points, args=(), daemon=True)
        self.recording_thread.start()

    # Stop recording
    def stop_record_action_points(self):
        self.is_recording = False


    # Replay the recording file
    def replay_recorded_data(self):
        self.control_cmd.enable_all_motor()
        with open(self.record_path) as f: 
            one_action_point = f.readline()
            while one_action_point:
                one_action_point = json.loads(one_action_point) 
                print(one_action_point)

                self.control_cmd.leg_motor_position_control(position = {"motor1":one_action_point["motor1"], 
                                                                        "motor2":one_action_point["motor2"], 
                                                                        "motor3":one_action_point["motor3"], 
                                                                        "motor4":one_action_point["motor4"], 
                                                                        "motor5":one_action_point["motor5"]})
                # time.sleep(0.1)
                one_action_point = f.readline()
    

    # Run the action in Pangolin_ActionGroups.py
    def run_action_curl(self, action_name = 'start_curl'):
        action = action_dic[action_name]
        for i in range(len(action)):
            self.control_cmd.leg_motor_position_control( position = {"motor1":action[i]["motor1"], 
                                                                     "motor2":action[i]["motor2"], 
                                                                     "motor3":action[i]["motor3"], 
                                                                     "motor4":action[i]["motor4"], 
                                                                     "motor5":action[i]["motor5"]})
            # print(i)
            time.sleep(1)

    def run_action_get_down_right(self, action_name = 'get_down_right'):
        action = action_dic[action_name]
        setPWMServoPulse(5, 1800, 100)
        setPWMServoPulse(6, 1300, 100)
        for i in range(len(action)):
            self.control_cmd.leg_motor_position_control(position = {"motor1":action[i]["motor1"], 
                                                                    "motor2":action[i]["motor2"], 
                                                                    "motor3":action[i]["motor3"], 
                                                                    "motor4":action[i]["motor4"], 
                                                                    "motor5":action[i]["motor5"]})
            # print(i)
            time.sleep(0.08)
        setPWMServoPulse(5, 1500, 100)
        setPWMServoPulse(6, 1700, 100)

    def run_action_get_down_left(self, action_name = 'get_down_left'):
        action = action_dic[action_name]
        setPWMServoPulse(5, 1200, 100)
        setPWMServoPulse(6, 1300, 100)
        for i in range(len(action)):
            self.control_cmd.leg_motor_position_control(position = {"motor1":action[i]["motor1"], 
                                                                    "motor2":action[i]["motor2"], 
                                                                    "motor3":action[i]["motor3"], 
                                                                    "motor4":action[i]["motor4"], 
                                                                    "motor5":action[i]["motor5"]})
            # print(i)
            time.sleep(0.08)
        setPWMServoPulse(5, 1500, 200)
        setPWMServoPulse(6, 1700, 200)    

    def run_action_stand_up_from_right(self, action_name = 'stand_up_from_right'):
        action = action_dic[action_name]
        for i in range(len(action)):
            self.control_cmd.leg_motor_position_control(position = {"motor1":action[i]["motor1"], 
                                                                    "motor2":action[i]["motor2"], 
                                                                    "motor3":action[i]["motor3"], 
                                                                    "motor4":action[i]["motor4"], 
                                                                    "motor5":action[i]["motor5"]})
            # print(i)
            time.sleep(1)

    def run_action_stand_up_from_left(self, action_name = 'stand_up_from_left'):
        action = action_dic[action_name]
        for i in range(len(action)):
            self.control_cmd.leg_motor_position_control(position = {"motor1":action[i]["motor1"], 
                                                                    "motor2":action[i]["motor2"], 
                                                                    "motor3":action[i]["motor3"], 
                                                                    "motor4":action[i]["motor4"], 
                                                                    "motor5":action[i]["motor5"]})
            # print(i)
            time.sleep(1)

    def stance_control(self):
        self.stance_cmd.reset_zero()
        self.stance_cmd.translation_x(self.x)
        self.stance_cmd.translation_z(self.z)
        self.stance_cmd.cal_pitch(self.pitch)
        self.stance_cmd.cal_roll(self.roll)
        # return self.stance_cmd.motor_pos
        self.control_cmd.leg_motor_position_control(position = {"motor1":self.inverse_kinematic(int(self.stance_cmd.motor_pos['motor1']*4095/360), 'motor1'), 
                                                                "motor2":self.inverse_kinematic(int(self.stance_cmd.motor_pos['motor2']*4095/360), 'motor2'), 
                                                                "motor3":self.inverse_kinematic(int(self.stance_cmd.motor_pos['motor3']*4095/360), 'motor3'), 
                                                                "motor4":self.inverse_kinematic(int(self.stance_cmd.motor_pos['motor4']*4095/360), 'motor4'),
                                                                "motor5":self.inverse_kinematic(int(self.stance_cmd.motor_pos['motor5']*4095/360), 'motor5')})
        
        return self.stance_cmd.motor_pos
    
    #Head
    def head_control(self, LeftRight, UpDown):
        setPWMServoPulse(5, int(LeftRight*300+1500), 100)
        setPWMServoPulse(6, int(UpDown*200+1500), 100)
        sleep(0.15)

class ControlCmd:
    def __init__(self): 

        #Record path
        self.record_path = 'output.txt'

        #Coummunicate the dynamixel motors
        self.dynamixel = DXL_Conmunication(DEVICE_NAME, B_RATE)
        self.dynamixel.activateDXLConnection()

        #Create the dynamixel motors
        motor1 = self.dynamixel.createMotor('motor1', motor_number=1)
        motor2 = self.dynamixel.createMotor('motor2', motor_number=2)
        motor3 = self.dynamixel.createMotor('motor3', motor_number=3)
        motor4 = self.dynamixel.createMotor('motor4', motor_number=4)
        motor5 = self.dynamixel.createMotor('motor5', motor_number=5)

        #Create the leg motor list
        self.leg_motor_list = [motor1, motor2, motor4, motor5]

        #Create the curl motor
        self.curl_motor = motor3

        self.motor_position = {"motor1":0, "motor2":0, "motor3":0, "motor4":0, "motor5":0}
        
        #Reboot and update the state
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()

        #Enalbe the motor torque
        self.enable_all_motor()

        #define the walk frequency
        self.walking_freq = 10

        #check list
        self.is_recording = False

        
    def enable_all_motor(self):
        for motor in self.leg_motor_list:
            motor.enableMotor()
        self.curl_motor.enableMotor()
        self.curl_motor.directWriteData(50, 112, 4)
            

    def disable_all_motor(self):
        for motor in self.leg_motor_list:
            motor.disableMotor()
        self.curl_motor.disableMotor()
        # self.dynamixel.closeHandler()
    
    # Read all the motors data
    def read_all_motor_data(self):
        self.dynamixel.updateMotorData()
        for motor in self.leg_motor_list:
            self.motor_position[motor.name] = motor.PRESENT_POSITION_value
        self.motor_position['motor3'] = self.curl_motor.PRESENT_POSITION_value
        #     self.motor_position[motor.name] = motor.PRESENT_CURRENT_value
        # self.motor_position['motor3'] = self.curl_motor.PRESENT_CURRENT_value

        print("motor_position", self.motor_position)
        return self.motor_position

    # Control led lights of all motors
    def motor_led_control(self, state = LED_ON):
        for motor in self.leg_motor_list:
            led_on = motor.directWriteData(state, *LED_ADDR_LEN)
        self.curl_motor.directWriteData(state, *LED_ADDR_LEN)
            
        return led_on
    
    
    # Control position of all motors
    def leg_motor_position_control(self, position = {"motor1":2000, "motor2":2000, "motor3":1025, "motor4":2000, "motor5":2000}):
        for motor in self.leg_motor_list:
            motor.writePosition(position[motor.name])
        self.curl_motor.writePosition(position["motor3"])
        self.dynamixel.sentAllCmd()
        time.sleep(1 / self.walking_freq)




if __name__ == "__main__":
    pangolin_control = PangolinControl()
    command_dict = {
        "enable":pangolin_control.control_cmd.enable_all_motor,
        "record":pangolin_control.start_record_action_points,
        "stop":pangolin_control.stop_record_action_points,
        "replay":pangolin_control.replay_recorded_data,
        "disable":pangolin_control.control_cmd.disable_all_motor,
        "read":pangolin_control.control_cmd.read_all_motor_data,
        "pos":pangolin_control.control_cmd.leg_motor_position_control,
        # "led":pangolin_control.start_led_blink,
        "curl":pangolin_control.run_action_curl,
        "getdown_right":pangolin_control.run_action_get_down_right,
        "getdown_left":pangolin_control.run_action_get_down_left,
        "standup_right":pangolin_control.run_action_stand_up_from_right,
        "standup_left":pangolin_control.run_action_stand_up_from_left,
        "reset":pangolin_control.reset_to_orginal,
        "stance":pangolin_control.stance_control,
    }


    while True:
        try:
            cmd = input("CMD : ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break
        except Exception as e:
            traceback.print_exc()
            break