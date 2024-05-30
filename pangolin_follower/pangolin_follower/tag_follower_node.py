import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray

class TagFollower(Node):
    def __init__(self):
        super().__init__('tag_follower')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_id = 0  # ID of the tag to follow
        self.linear_speed = 0.7  # Linear speed when moving forward
        self.angular_speed = 0.6  # Angular speed when turning
        self.center_threshold = 50  # Threshold for centering the tag

    def tag_callback(self, msg):
        if msg.detections:
            for detection in msg.detections:
                if detection.id == self.target_id:
                    self.follow_tag(detection)
                    return
        self.stop_robot()

    def follow_tag(self, detection):
        cmd = Twist()
        center_x = detection.centre.x

        image_center_x = 320  # Assuming 640x480 resolution

        # Proportional control for angular velocity
        error_x = center_x - image_center_x

        # Move forward if the tag is roughly centered
        if abs(error_x) < self.center_threshold:
            cmd.linear.x = self.linear_speed  # Move forward
            cmd.angular.z = 0.0  # No turning
        else:
            cmd.linear.x = 0.0  # Stop moving forward
            cmd.angular.z = -self.angular_speed * (error_x / abs(error_x))  # Turn towards the tag

        self.get_logger().info(f'linear.x:{cmd.linear.x}')
        self.get_logger().info(f'angular.z:{cmd.angular.z}')

        self.publisher.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TagFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
