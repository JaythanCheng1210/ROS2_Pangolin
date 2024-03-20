import rclpy
import os, sys, math

from rclpy.action import ActionClient
from std_srvs.srv import SetBool
from sensor_msgs.msg import Joy
from pangolin_interfaces.action import PangolinAction
from rclpy.node import Node

class PangolinActionClient(Node):
    def __init__(self):
        super().__init__('pangolin_action_client')
        self.action_client = ActionClient(self, PangolinAction, 'pangolin_action')
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.is_curl = False

    def joy_callback(self, msg):
        if msg.buttons[2] == 1:
            if not self.is_curl:  # Assuming you've a boolean flag to track the current state
                self.send_get_down_request()
                self.is_curl = True
            else:
                self.send_stand_up_request()
                self.is_curl = False

    def send_get_down_request(self):
        self.get_logger().info('Sending get down request...')
        goal_msg = PangolinAction.Goal()
        goal_msg.act_name = 'get_down'  # Assuming 'get_down' is the action name for Pangolin to get down
        self.action_client.wait_for_server()
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def send_stand_up_request(self):
        self.get_logger().info('Sending stand up request...')
        goal_msg = PangolinAction.Goal()
        goal_msg.act_name = 'stand_up'  # Assuming 'stand_up' is the action name for Pangolin to get down
        self.action_client.wait_for_server()
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted by server')
        else:
            self.get_logger().info('Goal rejected by server')

def main(args=None):
    rclpy.init(args=args)
    pangolin_action_client = PangolinActionClient()
    rclpy.spin(pangolin_action_client)
    pangolin_action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
