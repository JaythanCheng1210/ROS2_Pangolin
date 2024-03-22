from math import sin,cos,pi, asin, acos
import numpy as np
from Pangolin_Config import *
import traceback

class PangolinStance():
    def __init__(self):
        self.leg_height = LEG_HEIGHT
        self.l = L
        self.b = B
        self.w = W
        self.pitch_dz = 0
        self.roll_dz = 0

        self.motor_pos = {  "motor1":0, 
                            "motor2":0, 
                            "motor3":0,
                            "motor4":0,
                            "motor5":0}
    def reset_zero(self):
        self.motor_pos = {  "motor1":0, 
                            "motor2":0, 
                            "motor3":0,
                            "motor4":0,
                            "motor5":0}
        
    def translation_x(self, x = 40):
        try:
            rad = asin(x/self.leg_height)
            angle = rad*180/pi
        except:
            angle = 0
        # print('angle', angle)

        self.motor_pos["motor1"] += angle
        self.motor_pos["motor2"] += angle
        self.motor_pos["motor4"] += angle
        self.motor_pos["motor5"] += angle
    
    def z_to_deg(self, z):
        try:
            rad = acos(z/self.leg_height)
            angle = rad*180/pi
        except:
            angle = 0
        return angle
    
    def translation_z(self, z = 80):

        self.motor_pos["motor1"] += -self.z_to_deg(z + self.pitch_dz + self.roll_dz)
        self.motor_pos["motor2"] += -self.z_to_deg(z + self.pitch_dz - self.roll_dz)
        self.motor_pos["motor4"] += self.z_to_deg(z - self.pitch_dz + self.roll_dz)
        self.motor_pos["motor5"] += self.z_to_deg(z - self.pitch_dz - self.roll_dz)

        # self.motor_pos["motor1"] += self.z_to_deg(z)
        # self.motor_pos["motor2"] += self.z_to_deg(z)
        # self.motor_pos["motor4"] += self.z_to_deg(z)
        # self.motor_pos["motor5"] += self.z_to_deg(z)
        # print(self.motor_pos)
    
    
    def cal_pitch(self, angle = 5):
        self.pitch_dz = self.l/2 * sin(angle*(pi/180))

    def cal_roll(self, angle = 6):
        self.roll_dz = self.b /2* sin(angle*(pi/180))

    
if __name__ == '__main__':
    pangolin_stance = PangolinStance()
    command_dict = {
        "run_x":pangolin_stance.translation_x,
        "run_z":pangolin_stance.translation_z,
        "roll":pangolin_stance.cal_roll,
        "pitch":pangolin_stance.cal_pitch,
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