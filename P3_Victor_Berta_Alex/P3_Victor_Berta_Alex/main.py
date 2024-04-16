import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Color
import math

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
        # Centro del círculo
        x_center = self.vel_msg.linear.x
        y_center = self.vel_msg.linear.y

        x = radius
        y = 0
        err = 0

        while x >= y:
            self.move_turtle_to(x_center + x, y_center + y)
            self.move_turtle_to(x_center + y, y_center + x)
            self.move_turtle_to(x_center - y, y_center + x)
            self.move_turtle_to(x_center - x, y_center + y)
            self.move_turtle_to(x_center - x, y_center - y)
            self.move_turtle_to(x_center - y, y_center - x)
            self.move_turtle_to(x_center + y, y_center - x)
            self.move_turtle_to(x_center + x, y_center - y)

            y += 1
            err += 1 + 2*y
            if 2*(err-x) + 1 > 0:
                x -= 1
                err += 1 - 2*x

    def draw_grass(self, width, height):
        # Guarda el color actual de la tortuga
        original_color = self.get_turtle_color()

        # Cambia el color de la tortuga a verde
        green_color = Color(r=0, g=255, b=0)
        self.set_turtle_color(green_color)

        # Posición inicial del césped
        x_start = 0
        y_start = self.vel_msg.linear.y
        # Tamaño del césped
        grass_width = width
        grass_height = 2  # Altura del césped

        # Dibuja el césped en líneas horizontales
        for y in range(y_start, y_start + grass_height):
            self.move_turtle_to(x_start, y)
            self.move_turtle_to(x_start + grass_width, y)

        # Restaura el color original de la tortuga
        self.set_turtle_color(original_color)

    def draw_house(self):
        # Guarda el color actual de la tortuga
        original_color = self.get_turtle_color()

        # Cambia el color de la tortuga a rojo
        red_color = Color(r=255, g=0, b=0)
        self.set_turtle_color(red_color)

        # Dibuja el cuerpo de la casa
        self.draw_square(4)

        # Dibuja el techo de la casa
        self.draw_triangle(4)

        # Restaura el color original de la tortuga
        self.set_turtle_color(original_color)


    def draw_square(self, size):
        # Dibuja un cuadrado
        for _ in range(4):
            self.move_turtle_forward(size)
            self.turn_turtle_right(90)


    def draw_triangle(self, size):
        # Dibuja un triángulo
        for _ in range(3):
            self.move_turtle_forward(size)
            self.turn_turtle_right(120)

def main(args=None):
    rclpy.init(args=args)
    landscape_control = LandscapeControl()
    rclpy.spin(landscape_control)
    landscape_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
