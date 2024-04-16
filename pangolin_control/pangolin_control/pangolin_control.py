#！usr/bin python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from pangolin_interfaces.action import PangolinAction

import time
import os, sys, math
import numpy as np

import atexit
import threading


sys.path.append('/home/ubuntu/pangolin_ws/build/pangolin_control/driver')

from Pangolin_ControlCmd import PangolinControl
from Pangolin_ControlCmd import ControlCmd
from Pangolin_ActionGroups import action_dic
from Pangolin_Config import *
from Board import setPWMServoPulse

class Pangolin(Node):
    def __init__(self):
        super().__init__('pangolin_control')
        self.control_cmd = PangolinControl()
        # self.control_cmd2 = ControlCmd()

        self.joy_subscriber_ = self.create_subscription(Joy, 'joy', self.joy_callback, 0)
        self.imu_subscriber_ = self.create_subscription(Imu, 'imu', self.imu_callback, 1)
        self.cmd_vel_subscriber_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

        atexit.register(self.disable)
        
        
        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self.is_first_time = True
        self.is_disalbe_motor = False
        self.is_freedom_mode = False
        self.is_stance_mode = False
        self.is_record_mode = False
        self.is_sit_mode = False
        self.is_curl = False
        self.last_joy_msgs_buttons = []
        self.time_1 = 0

        #imu
        self.pitch = None
        self.roll = None
        self.yaw = None

        # atexit.register(self.cleanup)

    # destroy ros    
    def destroy(self):
        self.cmd_vel_subscriber_.destroy()
        self.imu_subscriber_.destroy()
        super().destroy_node()


    #Pangolin_action_server_
    # def handle_accepted_callback(self, goal_handle):
    #     with self._goal_lock:
    #         # This server only allows one goal at a time
    #         if self._goal_handle is not None and self._goal_handle.is_active:
    #             self.get_logger().info('Aborting previous goal')
    #             self._goal_handle.abort()
    #         self._goal_handle = goal_handle
    #     goal_handle.execute()

    # def pangolin_goal_callback(self, goal_request):
    #     """Accept or reject a client request to begin an action."""
    #     # This server allows multiple goals in parallel
    #     self.get_logger().info('Received goal request')
    #     return GoalResponse.ACCEPT
    
    # def pangolin_cancel_callback(self, goal_handle):
    #     """Accept or reject a client request to cancel an action."""
    #     self.get_logger().info('Received cancel request')
    #     return CancelResponse.ACCEPT
    
    # def pangolin_execute_callback(self, goal_handle):
    #     """Execute a goal."""
    #     self.get_logger().info('Executing goal...')
    #     feedback_msg = PangolinAction.Feedback()

    #     if goal_handle.request.act_name in action_dic:
    #         requested_action = goal_handle.request.act_name
    #         if requested_action == 'get_down':
    #             self.control_cmd.run_action_get_down()
    #         elif requested_action == "stand_up":
    #             self.control_cmd.run_action_stand_up()

    #         feedback_msg.which_action = requested_action

    #         self.get_logger().info("Publishing feedback: {0}".format(feedback_msg.which_action))
    #         goal_handle.Publish_feedback(feedback_msg)
    #         time.sleep(1)
        
    #     goal_handle.succeed()
    #     #Populate result message
    #     result = PangolinAction.Result()
    #     result.success = True
    #     self.get_logger().info('Returning result: {0}'.format(result.success))
    #     return result
    
    def joy_callback(self, msg):
        # X:0, A:1, B:2, Y:3
        # LB:4, RB:5, LT:6, RT:7
        # BACK:8, START:9, 
        # L3:10, R3:11  

        if self.is_first_time == True:
            self.last_joy_msgs_buttons = msg.buttons
            self.is_first_time = False

    # Stance control mode
        if msg.buttons[0] != self.last_joy_msgs_buttons[0]:
            self.is_stance_mode = not self.is_stance_mode

        if self.is_stance_mode == True:
            self.get_logger().info(f'Stance control mode')
            if msg.axes[1]>0.5:
                self.control_cmd.x += 1.5
            elif msg.axes[1]<-0.5:
                self.control_cmd.x -= 1.5
            if msg.axes[0]>0.5:
                self.control_cmd.z += 1.5
            elif msg.axes[0]<-0.5:
                self.control_cmd.z -= 1.5

            self.control_cmd.pitch = msg.axes[2]*15
            self.control_cmd.roll = msg.axes[3]*15

            # self.get_logger().info()
            # self.get_logger().info(f'button3: {self.control_cmd.stance_control()}')
        else:
            self.control_cmd.x = 0
            self.control_cmd.z = LEG_HEIGHT
            self.control_cmd.pitch = 0
            self.control_cmd.roll = 0
            
    # Reset mode
        if msg.buttons[1] != self.last_joy_msgs_buttons[1]:
            self.get_logger().info(f'Reset mode')
            self.control_cmd.reset_to_orginal()

    # Down & Curl action mode(left)
        if msg.buttons[6] != self.last_joy_msgs_buttons[6] and self.is_curl == False:
            self.get_logger().info(f'Curl action mode (left)')
            self.control_cmd.run_action_get_down_left()
            self.is_curl = True


    # Down & Curl action mode(right)
        if msg.buttons[7] != self.last_joy_msgs_buttons[7] and self.is_curl == False:
            self.get_logger().info(f'Curl action mode (right)')
            self.control_cmd.run_action_get_down_right()
            self.is_curl = True

        
        if msg.buttons[2] != self.last_joy_msgs_buttons[2] and self.is_curl == True:
            if self.pitch <= -65:
                self.control_cmd.run_action_stand_up_from_right()
                self.is_curl = False
            elif self.pitch >= 65:
                self.control_cmd.run_action_stand_up_from_left()
                self.is_curl = False


    # Freedom control mode
        if msg.buttons[3] != self.last_joy_msgs_buttons[3]:
            self.is_freedom_mode = not self.is_freedom_mode

        if self.is_freedom_mode == True:
            self.control_cmd.control_cmd.leg_motor_position_control(position = {"motor1":int(msg.axes[0]*1000 + self.control_cmd.motor_center_position["motor1"]), 
                                                                                "motor2":int(msg.axes[1]*1000 + self.control_cmd.motor_center_position["motor2"]), 
                                                                                "motor3":int(msg.axes[5]*1000 + self.control_cmd.motor_center_position["motor3"]),
                                                                                "motor4":int(msg.axes[2]*1000 + self.control_cmd.motor_center_position["motor4"]), 
                                                                                "motor5":int(msg.axes[3]*1000 + self.control_cmd.motor_center_position["motor5"])})

    # Record mode
        # if msg.buttons[4] != self.last_joy_msgs_buttons[4]:
        #     self.get_logger().info(f'record mode')
        #     if self.is_record_mode == False:
        #         self.control_cmd.start_record_action_points()
        #         self.is_record_mode = True
        #     else:
        #         self.control_cmd.stop_record_action_points()
        #         self.is_record_mode = False

        # if msg.buttons[5] != self.last_joy_msgs_buttons[5]:
        #     self.get_logger().info(f'replay')    
        #     self.control_cmd.replay_recorded_data()
  

        if msg.buttons[10] != self.last_joy_msgs_buttons[10]:
            self.is_sit_mode = not self.is_sit_mode

        if self.is_sit_mode == True:
            # self.control_cmd.control_cmd.leg_motor_position_control(position = {"motor1":2328, "motor2":1782, "motor3":1074, "motor4":1751, "motor5":2326 })
            self.control_cmd.run_action_sit()
        else:
            self.control_cmd.run_action_stand()





        self.last_joy_msgs_buttons = msg.buttons
        # self.get_logger().info(f'pitch: {self.pitch}')
        # self.get_logger().info(f'roll: {self.roll}')
        # self.get_logger().info(f'yaw: {self.yaw}')




# Pangolin cmd_vel callback
    def cmd_vel_callback(self, msg):

        # self.get_logger().info(f'linear.x: {msg.linear.x} angular.z: {msg.angular.z}')
        # self.control_cmd.set_servo_rate([msg.linear.x - msg.angular.z, msg.linear.x + msg.angular.z])

        if round(msg.linear.x, 0) != 0 and abs(msg.angular.z) < 0.5 and (self.is_curl == False) and (self.is_freedom_mode == False):
    
            self.control_cmd.set_servo_rate([msg.linear.x, msg.linear.x])
            if self.control_cmd.is_walking == False:
                self.get_logger().info(f'cmd_vel linear')
                self.control_cmd.set_gait_name('move_linear')
                self.control_cmd.start_gait()

        elif round(msg.angular.z, 0) < 0 and abs(msg.linear.x) < 0.5 and (self.is_curl == False) and (self.is_freedom_mode == False):
            
            self.control_cmd.set_servo_rate([msg.angular.z, msg.angular.z])
            if self.control_cmd.is_walking == False:
                self.get_logger().info(f'cmd_vel turn_right')
                self.control_cmd.set_gait_name('turn_right')
                self.control_cmd.start_gait()

        elif round(msg.angular.z, 0) > 0 and abs(msg.linear.x) < 0.5 and (self.is_curl == False) and (self.is_freedom_mode == False):
            
            self.control_cmd.set_servo_rate([-msg.angular.z, -msg.angular.z])
            if self.control_cmd.is_walking == False:
                self.get_logger().info(f'cmd_vel turn_left')
                self.control_cmd.set_gait_name('turn_left')
                self.control_cmd.start_gait()
        
        else:
            
            if self.control_cmd.is_walking == True:
                # self.get_logger().info(f'cmd_vel stop')
                self.control_cmd.stop_gait()
        
        # head control
        if (self.control_cmd.is_walking == False) and (self.control_cmd.is_turning == False) and (self.is_curl == False) and (self.is_stance_mode == False) and (self.is_freedom_mode == False):
            self.control_cmd.head_control(msg.linear.y, -(msg.linear.z))
        

#Pangolin imu callback
    def imu_callback(self, msg):
        
        # Get the imu msgs
        self.q0 = msg.orientation.x
        self.q1 = msg.orientation.y
        self.q2 = msg.orientation.z
        self.q3 = msg.orientation.w

        # Calculate eular angle
        self.pitch = -math.asin(-2*self.q1*self.q3+2*self.q0*self.q2)*57.3
        self.roll = math.atan2(2*self.q2*self.q3+2*self.q0*self.q1,-2*self.q1*self.q1-2*self.q2*self.q2+1)*57.3
        self.yaw = math.atan2(2*(self.q1*self.q2 + self.q0*self.q3),self.q0*self.q0+self.q1*self.q1-self.q2*self.q2-self.q3*self.q3)*57.3

        # self.get_logger().info(f'pitch: {pitch}')
        # self.get_logger().info(f'roll: {self.roll}')
        # self.get_logger().info(f'yaw: {yaw}')

        # Detect wether the robot has fallen over, then stand up.
        # if abs(roll) < 70 :
        #     self.time_1 = time.time()

        # elif abs(roll) >= 70:
        #     self.get_logger().info(f'imu stand up mode')
            
        #     if (time.time() - self.time_1 > 3):
        #         self.control_cmd.run_action_stand_up()
        #         self.is_curl = False

    # def cleanup(self):
    #     self.control_cmd2.disable_all_motor(self)

    def disable(self):
        self.control_cmd.disable_motor()

            

def main(args=None):
    rclpy.init(args=args)
    PangolinControl = Pangolin()

    rclpy.spin(PangolinControl)
    
    PangolinControl.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
