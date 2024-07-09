#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import curses
from std_msgs.msg import Header
import time
from art_msgs.msg import VehicleInput


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(VehicleInput, '/artcar_2/control/vehicle_inputs', 10)
        self.steering = 0.0
        self.throttle = 0.0
        self.speed_increment = 0.1
        self.steer_increment = 0.1

    def run(self):
        curses.wrapper(self.curses_loop)

    def curses_loop(self, stdscr):
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "ART Keyboard Teleop")
        stdscr.addstr(1, 0, "Use arrow keys to control the robot")
        stdscr.addstr(2, 0, "UP/DOWN: Throttle, LEFT/RIGHT: Steering")
        stdscr.addstr(3, 0, "X: Zero out inputs")
        stdscr.addstr(4, 0, "Q: Quit")

        while rclpy.ok():
            key = stdscr.getch()

            if key == curses.KEY_UP:
                self.throttle = min(1.0, self.throttle + self.speed_increment)
            elif key == curses.KEY_DOWN:
                self.throttle = max(0.0, self.throttle - self.speed_increment)
            elif key == curses.KEY_RIGHT:
                self.steering = max(-1.0, self.steering - self.steer_increment)
            elif key == curses.KEY_LEFT:
                self.steering = min(1.0, self.steering + self.steer_increment)
            elif key == ord('x'):
                self.steering = 0.0
                self.throttle = 0.0
            elif key == ord('q'):
                break

            self.publish_vehicle_input()
            stdscr.addstr(6, 0, f"Throttle: {self.throttle:.2f}   Steering: {self.steering:.2f}")
            stdscr.refresh()

    def publish_vehicle_input(self):
        msg = VehicleInput()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steering = self.steering
        msg.throttle = self.throttle
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

