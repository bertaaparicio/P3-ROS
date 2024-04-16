import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class LandscapeControl(Node):
    def __init__(self):
        super().__init__('landscape_control')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.draw_landscape)
        self.vel_msg = Twist()

    def draw_landscape(self):
        # Crea el círculo en la esquina superior derecha
        self.move_turtle_to(9, 9)
        self.draw_circle(2)

        # Crea el césped
        self.move_turtle_to(0, 5)
        self.draw_grass(10, 5)

        # Crea la casa
        self.move_turtle_to(5, 2)
        self.draw_house()

    def move_turtle_to(self, x, y):
        self.vel_msg.linear.x = x
        self.vel_msg.linear.y = y
        self.publisher_.publish(self.vel_msg)

    def draw_circle(self, radius):
        # Implementa el dibujo de un círculo aquí
        pass

    def draw_grass(self, width, height):
        # Implementa el dibujo del césped aquí
        pass

    def draw_house(self):
        # Implementa el dibujo de la casa aquí
        pass

def main(args=None):
    rclpy.init(args=args)
    landscape_control = LandscapeControl()
    rclpy.spin(landscape_control)
    landscape_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
