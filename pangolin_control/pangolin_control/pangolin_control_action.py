# ! usr/bin python3

import rclpy
from rclpy.node import Node
# from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
# from pangolin_interfaces.action import PangolinAction

import time
import os, sys, math
import numpy 
import threading

sys.path.append('/home/ubuntu/pangolin_ws/ros2-pangolin-robot/pangolin_control/driver')
sys.path.append('/home/ubuntu/pangolin_ws/ros2-pangolin-robot/pangolin_control/pangolin_control')

from Pangolin_ControlCmd import PangolinControl
from Pangolin_ActionGroups import action_dic
from Pangolin_Config import *
from Board import setPWMServoPulse
# from pangolin_control import Pangolin



class Pangolin_control_action(Node):
    def __init__(self):
        super().__init__('pangolin_action')
        self.control_cmd = PangolinControl()
        # self.pangolin_control = Pangolin()

        self.joy_subscriber_ = self.create_subscription(Joy, 'joy', self.joy_callback, 0)
        self.imu_subscriber_ = self.create_subscription(Imu, 'imu', self.imu_callback, 1)

        # self.pangolin_action_server_ = ActionServer( self, PangolinAction, 'pangolin_action', execute_callback=self.pangolin_execute_callback,
        #                                callback_group = ReentrantCallbackGroup(), goal_callback = self.pangolin_goal_callback,
        #                                handle_accepted_callback = self.handle_accepted_callback, cancel_callback = self.pangolin_cancel_callback)

        self._goal_lock = threading.Lock()
        self._goal_handle = None
        self.is_first_time = True
        self.is_disable_motor = False
        self.is_freedom_mode = False
        self.is_stance_mode = False
        self.is_record_mode = False
        self.is_curl = False
        self.last_joy_msgs_buttons = []
        self.time_1 = 0

        #IMU
        self.pitch = None
        self.roll = None
        self.yaw = None

    def destroy(self):
        self.joy_subscriber_.destroy()
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

    #Stance Control Mode
        if msg.buttons[0] != self.last_joy_msgs_buttons[0]:
            self.is_stance_mode = not self.is_stance_mode
        
        if self.is_stance_mode == True:
            self.get_logger().info(f'Stance Control Mode')
            if msg.axes[1] > 0.5:
                self.control_cmd.x += 1.5
            elif msg.axes[1] < -0.5:
                self.control_cmd.x -= 1.5
            
            if msg.axes[0] > 0.5:
                self.control_cmd.z += 1.5
            elif msg.axes[0] <- 0.5:
                self.control_cmd.z -= 1.5
            
            self.control_cmd.pitch = msg.axes[2]*15
            self.control_cmd.roll = msg.axes[3]*15
        
        else:
            self.control_cmd.x = 0
            self.control_cmd.z = LEG_HEIGHT
            self.control_cmd.pitch = 0
            self.control_cmd.roll = 0
            
    #Reset Mode
        if msg.butons[1] != self.last_joy_msgs_buttons[1]:
            self.get_logger().info(f"Reset Mode")
            self.control_cmd.reset_to_orginal()

    #Down & Curl Action Mode(left)
        if msg.button[6] != self.last_joy_msgs_buttons[6] and self.is_curl == False:
            self.get_logger().info(f"Curl Action Mode (left)")
            self.control_cmd.run_action_get_down_left()
            self.is_curl = True

    #Down & Curl Action Mode(right)
        if msg.button[7] != self.last_joy_msgs_buttons[7] and self.is_curl == False:
            self.get_logger().info(f"Curl Action Mode (right)")
            self.control_cmd.run_action_get_down_right()
            self.is_curl = True
        
    #Stand Up
        if msg.button[2] != self.last_joy_msgs_buttons[2] and self.is_curl == True:
            if self.roll >= 70:
                self.control_cmd.run_action_stand_up_from_right()
                self.is_curl = False
            elif self.roll <= -70:
                self.control_cmd.run_action_stand_up_from_left()
                self.is_curl = False
    
    #Freedom Control Mode
        if msg.button[3] != self.last_joy_msgs_buttons[3]:
            self.is_freedom_mode = not self.is_freedom_mode
        
        if self.is_freedom_mode == True:
            self.control_cmd.control_cmd.leg_motor_position_control(position = {"motor1":int(msg.axes[0]*1000 + self.control_cmd.motor_center_position["motor1"]), 
                                                                                "motor2":int(msg.axes[1]*1000 + self.control_cmd.motor_center_position["motor2"]), 
                                                                                "motor3":0, 
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
  

        if msg.buttons[8] != self.last_joy_msgs_buttons[8]:
            pass

        self.last_joy_msgs_buttons = msg.buttons
        # self.get_logger().info(f'roll: {self.roll}')



#Pangolin IMU callback
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


def main(args=None):
    rclpy.init(args=args)
    PangolinControlAction = Pangolin_control_action()

    rclpy.spin(PangolinControlAction)
    
    PangolinControlAction.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()