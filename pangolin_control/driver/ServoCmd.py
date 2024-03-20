import sys
import threading
# from PWMServoControl import *
from time import sleep
from Board import setPWMServoPulse
# from multi_robot_datatype import BatteryState, OverViewState, JointStates, UWBState, Vector3, Twist
# import ActionGroupControl as AGC

# servo = PWMServo()

# servo.setThreshold(1,500,2500)
# servo.setThreshold(2,500,2500)
# servo.setThreshold(3,500,2500)
# servo.setThreshold(4,500,2500) 

# def getServoPulse(id):
#     return servo.servo_pwm_duty_now[id]

# def getServoDeviation(id):
#     return servo.getDeviation(id)

# def setServoPulse(id, pulse, use_time):
#     servo.setPulse(id, pulse, use_time)

# def setServoDeviation(id ,dev):
#     servo.setDeviation(id, dev)
    
# def saveServoDeviation(id):
#     servo.saveDeviation(id)

# def unloadServo(id):
#     servo.unload(id)

# def updatePulse(id):
#     servo.updatePulse(id)

def default():
    setPWMServoPulse(5, 1500, 100)
    setPWMServoPulse(6, 1500, 100)

def test():
    setPWMServoPulse(5, 1000, 100)
    setPWMServoPulse(6, 1300, 100)
    sleep(1)
    setPWMServoPulse(5, 2000, 100)
    setPWMServoPulse(6, 1700, 100)
    sleep(1)
    setPWMServoPulse(5, 1000, 100)
    setPWMServoPulse(6, 1300, 100)
    sleep(1)
    setPWMServoPulse(5, 2000, 100)
    setPWMServoPulse(6, 1700, 100)
    sleep(1)
    setPWMServoPulse(5, 1000, 100)
    setPWMServoPulse(6, 1300, 100)
    sleep(1)
    setPWMServoPulse(5, 2000, 100)
    setPWMServoPulse(6, 1700, 100)
    sleep(1)


if __name__ == "__main__":
    try:
        while(True):
            test()
    except KeyboardInterrupt:
        sleep(0.2)
        default()
        pass