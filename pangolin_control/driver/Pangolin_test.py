from DXL_motor_control import DXL_Conmunication
import time
import sys, tty, termios
import traceback
import json 
import threading
import log

# Keyboard interrupt 
# fd = sys.stdin.fileno()
# old_settings = termios.tcgetattr(fd)
# def getch():
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch


# main control command
DEVICE_NAME = "/dev/ttyUSB0"
B_RATE      = 57600
LED_ADDR_LEN = (65,1)
LED_ON = 1
LED_OFF = 0

class PangolinControl:
    def __init__(self, log_level="info", log_file_level="debug"):
        self.control_cmd = ControlCmd()

        self.log_level = log_level
        self.log_file_level = log_file_level
        self.log = log.LogHandler(self.__class__.__name__, __name__, self.log_level, self.log_file_level)

        self.motor_center_position = {"motor1":1423, "motor2":2672, "motor3": 0, "motor4":2672, "motor5":1423}
        self.control_cmd.leg_motor_position_control(position = {"motor1":self.motor_center_position["motor1"]    ,"motor2":self.motor_center_position["motor2"]      , "motor3":0, "motor4":self.motor_center_position["motor4"]     , "motor5":self.motor_center_position["motor5"]     })
        self.init_fail = False
        self.is_walking = False
        self.servo_rate = [1.0, 1.0]

        self.is_led_blink = True
        self.is_recording = False
        self.record_path = '/home/ubuntu/pangolin_robot_ws/ros2-pangolin-robot/pangolin_control/driver/output.txt'


    def reset_to_orginal(self):    
        self.control_cmd.leg_motor_position_control(position = {"motor1":self.motor_center_position["motor1"]    ,"motor2":self.motor_center_position["motor2"]      , "motor3":0, "motor4":self.motor_center_position["motor4"]     , "motor5":self.motor_center_position["motor5"]     })

    def start_thread(self, thread_name):
        if thread_name == "process_gait":
            self.is_walking = True
            thread = threading.Thread(target=self.process_gait, args=(), daemon=True)




        self.walking_thread.start()


    def process_gait(self):
        while self.is_walking:
            self.control_cmd.leg_motor_position_control(position = {"motor1":int( 200*self.servo_rate[0]+self.motor_center_position["motor1"]), "motor2":int(+300*self.servo_rate[1]+self.motor_center_position["motor2"]) , "motor3":0, "motor4":int(-300*self.servo_rate[0]+self.motor_center_position["motor4"]) , "motor5":int(-200*self.servo_rate[1]+self.motor_center_position["motor5"]) })
            self.control_cmd.leg_motor_position_control(position = {"motor1":int( 200*self.servo_rate[0]+self.motor_center_position["motor1"]), "motor2":int(     self.servo_rate[1]+self.motor_center_position["motor2"]) , "motor3":0, "motor4":int(     self.servo_rate[0]+self.motor_center_position["motor4"]) , "motor5":int(-200*self.servo_rate[1]+self.motor_center_position["motor5"]) })
            self.control_cmd.leg_motor_position_control(position = {"motor1":int(-300*self.servo_rate[0]+self.motor_center_position["motor1"]), "motor2":int(     self.servo_rate[1]+self.motor_center_position["motor2"]) , "motor3":0, "motor4":int(     self.servo_rate[0]+self.motor_center_position["motor4"]) , "motor5":int(+300*self.servo_rate[1]+self.motor_center_position["motor5"]) })
            self.control_cmd.leg_motor_position_control(position = {"motor1":int(-300*self.servo_rate[0]+self.motor_center_position["motor1"]), "motor2":int(-200*self.servo_rate[1]+self.motor_center_position["motor2"]) , "motor3":0, "motor4":int(+200*self.servo_rate[0]+self.motor_center_position["motor4"]) , "motor5":int(+300*self.servo_rate[1]+self.motor_center_position["motor5"]) })
            self.control_cmd.leg_motor_position_control(position = {"motor1":int(     self.servo_rate[0]+self.motor_center_position["motor1"]), "motor2":int(-200*self.servo_rate[1]+self.motor_center_position["motor2"]) , "motor3":0, "motor4":int(+200*self.servo_rate[0]+self.motor_center_position["motor4"]) , "motor5":int(     self.servo_rate[1]+self.motor_center_position["motor5"]) })
            self.control_cmd.leg_motor_position_control(position = {"motor1":int(     self.servo_rate[0]+self.motor_center_position["motor1"]), "motor2":int(+300*self.servo_rate[1]+self.motor_center_position["motor2"]) , "motor3":0, "motor4":int(-300*self.servo_rate[0]+self.motor_center_position["motor4"]) , "motor5":int(     self.servo_rate[1]+self.motor_center_position["motor5"]) })


    def start_gait(self):
        self.is_walking = True
        self.walking_thread = threading.Thread(target=self.process_gait, args=(), daemon=True)
        self.walking_thread.start()


    def stop_gait(self):
        self.is_walking = False
        self.reset_to_orginal()


    def set_servo_rate(self, servo_rate=[1.0, 1.0]):
        self.servo_rate = servo_rate

    # start recording the motors position until pressing ESC
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
            time.sleep(1)
            self.control_cmd.motor_led_control(LED_OFF)
        print("finish recording!")


    def start_record_action_points(self):
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self.process_record_action_points, args=(), daemon=True)
        self.recording_thread.start()

    # start recording the motors position until pressing ESC
    def stop_record_action_points(self):
        self.is_recording = False


    # Replay the recording file
    def replay_motor_data(self):
        self.control_cmd.enable_all_motor()
        with open(self.record_path) as f: 
            one_action_point = f.readline()
            while one_action_point:
                one_action_point = json.loads(one_action_point) 
                print(one_action_point)

                self.control_cmd.leg_motor_position_control(position = {"motor1":one_action_point["motor1"], "motor2":one_action_point["motor2"], "motor3":0, 
                                                                        "motor4":one_action_point["motor4"], "motor5":one_action_point["motor5"]})
                time.sleep(1)
                one_action_point = f.readline()
    

    # def motor_led_blink(self):
    #     while self.is_led_blink:
    #         self.control_cmd.motor_led_control(LED_ON)
    #         time.sleep(0.2)
    #         self.control_cmd.motor_led_control(LED_OFF)
    #         time.sleep(0.2)
    
    # def start_led_blink(self):
    #     self.is_led_blink = True
    #     self.led_blinking_thread = threading.Thread(target=self.motor_led_blink, args=(), daemon=True)
    #     self.led_blinking_thread.start()

    # def stop_led_blink(self):
    #     self.is_led_blink = False


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

            # self.motor_position[motor.name] = int(motor.PRESENT_POSITION_value*360/4095)  #360 degrees
        print("motor_position", self.motor_position)
        return self.motor_position

    # Control led lights of all motors
    def motor_led_control(self, state = LED_ON):
        for motor in self.leg_motor_list:
            led_on = motor.directWriteData(state, *LED_ADDR_LEN)
        self.curl_motor.directWriteData(state, *LED_ADDR_LEN)
            
        return led_on
    
    
    # Control position of all motors
    def leg_motor_position_control(self, position = {"motor1":2000, "motor2":2000, "motor3":2000, "motor4":2000, "motor5":2000}):
        for motor in self.leg_motor_list:
            motor.writePosition(position[motor.name])
        self.dynamixel.sentAllCmd()
        time.sleep(1 / self.walking_freq)

    def start_recording(self):
        self.is_recording = True
        # record = True
        self.recording_thread = threading.Thread(target=self.start_record_action_points, args=(), daemon=True)
        self.recording_thread.start()

    # start recording the motors position until pressing ESC
    def start_record_action_points(self):
        # self.is_recording = record
        self.disable_all_motor()
        self.dynamixel.rebootAllMotor()
        print("start record the action points....")
        with open(self.record_path, 'w') as f:
            print("start record the action points....")
            while self.is_recording:
                all_servo_position = self.read_all_motor_data()
                print("recording:", all_servo_position)
                f.write(json.dumps(all_servo_position)+'\n')
                time.sleep(1)
        print("finish recording!")

    # start recording the motors position until pressing ESC
    def stop_record_action_points(self):
        self.is_recording = False


    # Replay the recording file
    def replay_motor_data(self):
        self.enable_all_motor()
        with open(self.record_path) as f: 
            one_action_point = f.readline()
            while one_action_point:
                one_action_point = json.loads(one_action_point) 
                print(one_action_point)

                for motor in self.leg_motor_list:
                    motor.writePosition(one_action_point[motor.name])
                # self.curl_motor.writePosition(one_action_point['motor3'])
                self.dynamixel.sentAllCmd()
                time.sleep(1)
                one_action_point = f.readline()



if __name__ == "__main__":
    # controlcmd = ControlCmd()
    pangolin_control = PangolinControl()
    # while True:
    #     print("Press any key to continue! (or press ESC to quit!)")
    #     if getch() == chr(0x1b):
    #         break
    #     all_servo_position = controlcmd.read_motor_data()
    #     print(all_servo_position)
    # controlcmd.disable_all_motor()

    command_dict = {
        "enable":pangolin_control.control_cmd.enable_all_motor,
        "record":pangolin_control.start_record_action_points,
        "stop":pangolin_control.stop_record_action_points,
        "replay":pangolin_control.control_cmd.replay_motor_data,
        "disable":pangolin_control.control_cmd.disable_all_motor,
        "read":pangolin_control.control_cmd.read_all_motor_data,
        "pos":pangolin_control.control_cmd.leg_motor_position_control,
        "led":pangolin_control.start_led_blink,
        "run":pangolin_control.start_gait,


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