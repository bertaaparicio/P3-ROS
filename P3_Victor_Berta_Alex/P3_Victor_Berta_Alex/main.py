import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from rclpy.qos import QoSProfile
import math


class DrawLandscape:
    def __init__(self):
        self.node = rclpy.create_node('draw_landscape')
        self.publisher = self.node.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        qos = QoSProfile(depth=10)
        self.subscription = self.node.create_subscription(Pose, 'turtle2/pose', self.pose_callback, 10)
        self.subscription

        self.spawn_turtle()

    def spawn_turtle(self):
        client = self.node.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        request = Spawn.Request()
        request.x = 1.0
        request.y = 1.0
        request.theta = 0.0
        request.name = 'turtle2'
        future = client.call_async(request)

    def pose_callback(self, data):
        # Calculate the direction to move towards the circle center
        circle_direction = math.atan2(5.0 - data.y, 5.0 - data.x)

        # Calculate the direction to move towards the line start point
        line_direction = math.atan2(2.0 - data.y, 1.0 - data.x)

        # Calculate the distance to the circle center
        circle_distance = math.sqrt((5.0 - data.x)**2 + (5.0 - data.y)**2)

        # Calculate the distance to the line
        line_distance = math.sqrt((9.0 - 2.0)**2 + (8.0 - 2.0)**2)

        # Stop the turtle if it's close to the circle center or line start point
        if circle_distance < 0.1 or line_distance < 0.1:
            twist = Twist()
            self.publisher.publish(twist)
        else:
            # Calculate the angular velocity to move towards the circle center
            twist_circle = Twist()
            twist_circle.linear.x = 1.0  # adjust linear velocity as needed
            twist_circle.angular.z = self.calculate_angular_velocity(data.theta, circle_direction)
            self.publisher.publish(twist_circle)

            # Calculate the angular velocity to move towards the line start point
            twist_line = Twist()
            twist_line.linear.x = 1.0  # adjust linear velocity as needed
            twist_line.angular.z = self.calculate_angular_velocity(data.theta, line_direction)
            self.publisher.publish(twist_line)

        self.rate.sleep()  # maintain the rate of publishing commands

    def calculate_angular_velocity(self, current_angle, target_angle):
        # Calculate the shortest angular distance to the target angle
        delta_angle = target_angle - current_angle
        if delta_angle > math.pi:
            delta_angle -= 2 * math.pi
        elif delta_angle < -math.pi:
            delta_angle += 2 * math.pi

        # Convert the angular distance to angular velocity
        return delta_angle * 2

def main(args=None):
    rclpy.init(args=args)
    draw_landscape = DrawLandscape()
    rclpy.spin(draw_landscape.node)
    draw_landscape.node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()