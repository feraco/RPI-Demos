import json
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import math
import rclpy  # ROS2 Python library
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsState, MotorState

class LandBotSimulator():
    def __init__(self, real=False):
        self.real_mode = real
        self._init_state()
        if self.real_mode:
            rclpy.init()
            self.node = Node('landbot_controller')
            self.motor_pub = self.node.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
            print("LandBot Real Mode Enabled: Sending commands to ROS2")
        else:
            print("LandBot Simulator Initialized")
    @staticmethod
    def serialize_command(command: dict):
        """Converts a command dictionary to a string format."""
        serialized = command['command']
        if command.get('arguments'):
            serialized = f"{serialized} {' '.join(map(str, command['arguments']))}"
        return serialized

    def _init_state(self):
        self.position = (0, 0)  # X, Y coordinates
        self.bearing = 0  # Yaw in degrees
        self.path_coors = [(0, 0)]
        self.yaw_data = [0]
        self.command_log = []

    def send_command(self, command: str, *args):
        command_json = {'command': command, 'arguments': args}
        self.command_log.append(command_json)
        print(f'Executing command: {self.serialize_command(command_json)}')
        if self.real_mode:
            self.publish_motor_command(command, *args)
        time.sleep(1)

    def publish_motor_command(self, command, *args):
        motor_states = []
        speed = args[0] if args else 0.5  # Default speed if not provided
        if command == 'move_forward':
            motor_states = [MotorState(id=i, rps=speed) for i in range(1, 5)]
        elif command == 'move_backward':
            motor_states = [MotorState(id=i, rps=-speed) for i in range(1, 5)]
        elif command == 'move_left':
            motor_states = [MotorState(id=1, rps=-speed), MotorState(id=2, rps=speed), MotorState(id=3, rps=speed), MotorState(id=4, rps=-speed)]
        elif command == 'move_right':
            motor_states = [MotorState(id=1, rps=speed), MotorState(id=2, rps=-speed), MotorState(id=3, rps=-speed), MotorState(id=4, rps=speed)]
        elif command == 'rotate':
            angular_speed = args[0] if args else 0.5
            motor_states = [MotorState(id=1, rps=-angular_speed), MotorState(id=2, rps=-angular_speed), MotorState(id=3, rps=angular_speed), MotorState(id=4, rps=angular_speed)]
        if motor_states:
            msg = MotorsState(data=motor_states)
            self.motor_pub.publish(msg)
            print(f"Published motor command: {command}")
    
    def move_forward(self, speed, duration):
        self.send_command('move_forward', speed)
        self.move(speed * np.cos(np.radians(self.bearing)), speed * np.sin(np.radians(self.bearing)), duration)
    
    def move_backward(self, speed, duration):
        self.send_command('move_backward', speed)
        self.move(-speed * np.cos(np.radians(self.bearing)), -speed * np.sin(np.radians(self.bearing)), duration)

    def move_left(self, speed, duration):
        self.send_command('move_left', speed)
        self.move(-speed, 0, duration)
    
    def move_right(self, speed, duration):
        self.send_command('move_right', speed)
        self.move(speed, 0, duration)
    
    def rotate_left(self, degrees):
        self.rotate(-degrees)
    
    def rotate_right(self, degrees):
        self.rotate(degrees)

    def rotate(self, degrees):
        self.bearing = (self.bearing + degrees) % 360
        self.yaw_data.append(self.bearing)
        self.send_command('rotate', degrees)
        self.plot_yaw()
        self.plot_bearing()

    def plot_flight_path(self):
        fig, ax = plt.subplots()
        df = pd.DataFrame(self.path_coors, columns=['X', 'Y'])
        ax.plot(df['X'], df['Y'], 'bo-', linewidth=2, markersize=8)
        ax.scatter(df.iloc[-1]['X'], df.iloc[-1]['Y'], c='red', marker='^', s=100, label='LandBot')
        ax.grid()
        ax.set(xlabel='X Position', ylabel='Y Position', title='LandBot Movement Path')
        ax.legend()
        plt.show()

    def plot_yaw(self):
        plt.figure()
        plt.plot(self.yaw_data, 'g-', marker='o')
        plt.xlabel('Step')
        plt.ylabel('Yaw (Degrees)')
        plt.title('Yaw Over Time')
        plt.grid()
        plt.show()

    def plot_bearing(self):
        plt.figure()
        plt.plot(range(len(self.yaw_data)), self.yaw_data, 'm-', marker='s')
        plt.xlabel('Step')
        plt.ylabel('Bearing (Degrees)')
        plt.title('LandBot Movement Direction')
        plt.grid()
        plt.show()

    def reset(self):
        print("Resetting LandBot simulation...")
        self._init_state()

    def save(self, file_path='landbot_commands.json'):
        with open(file_path, 'w') as json_file:
            json.dump(self.command_log, json_file, indent=4)
        print(f"Commands saved to {file_path}")

    def load_commands(self, file_path):
        with open(file_path) as json_file:
            commands = json.load(json_file)
        self._init_state()
        for command in commands:
            getattr(self, command['command'])(*command['arguments'])

if __name__ == '__main__':
    landbot = LandBotSimulator(real=True)  # Change to False for simulation
    landbot.move_forward(1.0, 2.0)
    landbot.rotate_right(90)
    landbot.move_forward(1.0, 2.0)
    landbot.move_left(0.5, 1.5)
    landbot.rotate_left(45)
    landbot.move_backward(1.0, 2.0)
