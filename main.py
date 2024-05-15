import time
import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen, TeleportAbsolute
from rclpy.qos import QoSProfile
import math


class DrawLandscape:
    def __init__(self):
        self.node = rclpy.create_node('draw_landscape')
        self.publisher = self.node.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        qos = QoSProfile(depth=10)
        self.subscription = self.node.create_subscription(Pose, 'turtle2/pose', self.pose_callback, 10)

        self.circle_drawn = False
        self.spawn_turtle()

        self.set_pen_color_and_width2()
        self.teleport_turtle(0.0, 2.0)
        self.teleport_turtle(11.0, 2.0)
        self.teleport_turtle(5.54, 5.54)

        self.spawn_third_turtle()
        self.teleport_third_turtle(2.5, 3.0)
        self.teleport_third_turtle(2.5, 1.0)
        self.teleport_third_turtle(2.5, 2.25)
        self.teleport_third_turtle(3.0, 2.25)
        self.teleport_third_turtle(3.0, 2.75)

        time.sleep(0.7)
        self.destroy_turtle('turtle1')
        self.destroy_turtle('turtle2')
        self.destroy_turtle('turtle3')

    def destroy_turtle(self, turtle_name):
        client = self.node.create_client(Kill, 'kill')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Fatal')
        request = Kill.Request()
        request.name = turtle_name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)


    def spawn_turtle(self):
        client = self.node.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Fatal')
        request = Spawn.Request()
        request.x = 9.0
        request.y = 10.0
        request.theta = 0.0
        request.name = 'turtle2'
        future = client.call_async(request)
        future.add_done_callback(self.set_pen_color_and_width)

    def pose_callback(self, data):
        if not self.circle_drawn:
            circle_direction = math.atan2(5.0 - 9, 5.0 - 10)

            circle_distance = math.sqrt((5.0 - 9) ** 2 + (5.0 - 10) ** 2)

            if circle_distance < 0.1:
                twist = Twist()
                self.publisher.publish(twist)
                self.circle_drawn = True
            else:
                twist_circle = Twist()
                twist_circle.linear.x = 1.0
                twist_circle.angular.z = self.calculate_angular_velocity(0, circle_direction)
                self.publisher.publish(twist_circle)

    def calculate_angular_velocity(self, current_angle, target_angle):
        delta_angle = target_angle - current_angle
        if delta_angle > math.pi:
            delta_angle -= 2 * math.pi
        elif delta_angle < -math.pi:
            delta_angle += 2 * math.pi
        return delta_angle * 2

    def teleport_turtle(self, x, y):
        client = self.node.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Fatal')
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)



    def set_pen_color_and_width(self, future):
        response = future.result()
        if response is not None:
            pen_client = self.node.create_client(SetPen, 'turtle2/set_pen')
            while not pen_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('Fatal')
            request = SetPen.Request()
            request.r = 255
            request.g = 255
            request.b = 0
            request.width = 50
            request.off = 0
            pen_future = pen_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, pen_future)


    def set_pen_color_and_width2(self):
        pen_client = self.node.create_client(SetPen, 'turtle1/set_pen')
        while not pen_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Fatal')
        request = SetPen.Request()
        request.r = 200
        request.g = 80
        request.b = 20
        request.width = 15
        request.off = 0
        pen_future = pen_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, pen_future)


    def set_pen_color_and_width3(self, future):
        response = future.result()
        if response is not None:
            pen_client = self.node.create_client(SetPen, 'turtle3/set_pen')
            while not pen_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('Fatal')
            request = SetPen.Request()
            request.r = 0
            request.g = 120
            request.b = 0
            request.width = 9
            request.off = 0
            pen_future = pen_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, pen_future)


    def spawn_third_turtle(self):
        client = self.node.create_client(Spawn, 'spawn')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Fatal')
        request = Spawn.Request()
        request.x = 2.5
        request.y = 1.0
        request.theta = 0.0
        request.name = 'turtle3'
        future = client.call_async(request)
        future.add_done_callback(self.set_pen_color_and_width3)

    def teleport_third_turtle(self, x, y):
        client = self.node.create_client(TeleportAbsolute, 'turtle3/teleport_absolute')
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Fatal')
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)



def main(args=None):
    rclpy.init(args=args)
    draw_landscape = DrawLandscape()
    rclpy.spin(draw_landscape.node)
    draw_landscape.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
