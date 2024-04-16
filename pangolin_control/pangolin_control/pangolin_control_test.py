#！usr/bin python3

import rclpy
from rclpy.node import Node
import time
import threading
import traceback
import os, sys, math
from rclpy.executors import SingleThreadedExecutor
sys.path.append('/home/ubuntu/pangolin_ws/build/pangolin_control/driver')
from Pangolin_Config import *
from Pangolin_ControlCmd import PangolinControl
from Board import setPWMServoPulse
from Pangolin_ActionGroups import action_dic

class TaskHandler(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.control_cmd = PangolinControl()
        self.time_is_up = False
        self.timer_is_running = False
        self.is_start = False

    def curlLeft(self):
        self.control_cmd.run_action_get_down_left()

    def curlRight(self):
        self.control_cmd.run_action_get_down_right()
    
    def standUpLeft(self):
        self.control_cmd.run_action_stand_up_from_left()
    
    def standUpRight(self):
        self.control_cmd.run_action_stand_up_from_right()
    
    def disable(self):
        self.is_start = False
        self.control_cmd.control_cmd.disable_all_motor()
    
    def walk(self):
        if self.control_cmd.is_walking == False:
            self.control_cmd.set_gait_name('move_linear')
            self.control_cmd.start_gait()
        
    def turnRight(self):
        if self.control_cmd.is_walking == False:
            self.get_logger().info(f'cmd_vel turn_right')
            self.control_cmd.set_gait_name('turn_right')
            self.control_cmd.start_gait()
    
    def turnLeft(self):
        if self.control_cmd.is_walking == False:
            self.get_logger().info(f'cmd_vel turn_left')
            self.control_cmd.set_gait_name('turn_left')
            self.control_cmd.start_gait()
    
    def startLoop(self):
        self.is_start = True
        self.loop_thread = threading.Thread(target=self.test, daemon=True)
        self.loop_thread.start()
        
    def test(self):
        for _ in range(100):
            self.walk()
            time.sleep(5)
            self.control_cmd.stop_gait()
            self.curlLeft()
            time.sleep(3)
            self.standUpLeft()
            time.sleep(1)
            self.turnRight()
            time.sleep(15)
            self.control_cmd.stop_gait()
            self.curlRight()
            time.sleep(3)
            self.standUpRight()
            time.sleep(1)
            self.walk()
            time.sleep(5)
            self.control_cmd.stop_gait()
            self.curlRight()
            time.sleep(3)
            self.standUpRight()
            time.sleep(1)
            self.turnLeft()
            time.sleep(15)
            self.control_cmd.stop_gait()
            self.curlLeft()
            time.sleep(3)
            self.standUpLeft()
            time.sleep(1)
            print("Cycle done !!")

    def startTimer(self, set_time, motion):
        self.time_is_up = False
        if motion == 'walk':
            self.walk()
        elif motion == 'down':
            self.curlLeft()
            time.sleep(3)
        self.timer_thread = threading.Thread(target=self.timerLoop, args=(set_time, motion,),daemon=True)
        self.timer_thread.start()
    
    def timerLoop(self, set_time, motion):
        self.timer_is_running = True
        start_time = time.time()
        while self.timer_is_running:
            time_duration = time.time() - start_time
            if time_duration >= set_time:
                self.time_is_up = True
                if motion == 'walk':
                    self.control_cmd.stop_gait()
                elif motion == 'down':
                    self.standUpLeft()
                break
            time.sleep(0.01)

    def stopTimer(self):
        self.timer_is_running = False
        

    def destroy(self):
        super().destroy_node()

def handleInterrupt(task_handler):
    excepthook = sys.excepthook
    def new_hook(exctype, value, traceback):
        if exctype != KeyboardInterrupt:
            excepthook(exctype, value, traceback)
        else:
            task_handler.disable()
    sys.excepthook = new_hook

def main(task_handler):
    # rclpy.init()
    # task_handler = TaskHandler()

    command_dict = {
        "down": task_handler.curlLeft,
        "up": task_handler.standUpLeft,
        "test": task_handler.test,
    }
    # while True:
    try:
        # cmd = input("CMD : ")
        # if cmd in command_dict:
        #     command_dict[cmd]()
        # elif cmd == "exit":
        #     task_handler.disable()
        #     break
        task_handler.test()
    except Exception as e:
        task_handler.disable()
        traceback.print_exc()
        # break

    
    task_handler.destroy()
    # rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    task_handler = TaskHandler()
    handleInterrupt(task_handler)
    main(task_handler)
    rclpy.shutdown()