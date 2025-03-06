import json
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import math

class LandBotSimulator():
    def __init__(self, real=False):
        self.real_mode = real
        self._init_state()
        if self.real_mode:
            print("LandBot Real Mode Enabled: Sending commands (ROS2 not available in cloud version)")
        else:
            print("LandBot Simulator Initialized (Cloud Mode)")

    @staticmethod
    def serialize_command(command: dict):
        """Converts a command dictionary to a string format."""
        serialized = command['command']
        if command.get('arguments'):
            serialized = f"{serialized} {' '.join(map(str, command['arguments']))}"
        return serialized

    def _init_state(self):
        self.position = (0, 0)
        self.bearing = 0
        self.path_coors = [(0, 0)]
        self.yaw_data = [0]
        self.command_log = []

    def send_command(self, command: str, *args):
        command_json = {'command': command, 'arguments': args}
        self.command_log.append(command_json)
        print(f'Executing command: {LandBotSimulator.serialize_command(command_json)}')
        time.sleep(1)

    def move(self, vx, vy, duration):
        new_x = self.position[0] + vx * duration
        new_y = self.position[1] + vy * duration
        self.position = (new_x, new_y)
        self.path_coors.append((new_x, new_y))
        self.plot_flight_path()

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

    def move_in_direction(self, degrees, speed, duration):
        radians = np.radians(degrees)
        vx = speed * np.cos(radians)
        vy = speed * np.sin(radians)
        self.send_command('move_in_direction', degrees, speed, duration)
        self.move(vx, vy, duration)

    def rotate_right(self, degrees):
        self.rotate(degrees)

    def rotate_left(self, degrees):
        self.rotate(-degrees)

    def rotate(self, degrees):
        self.bearing = (self.bearing + degrees) % 360
        self.yaw_data.append(self.bearing)
        self.send_command('rotate', degrees)
        self.plot_yaw()
        self.plot_bearing()

    def plot_flight_path(self):
        fig, ax = plt.subplots()
        df = pd.DataFrame(self.path_coors, columns=['X', 'Y'])
        ax.plot(df['X'].to_numpy(), df['Y'].to_numpy(), 'bo-', linewidth=2, markersize=8)
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

if __name__ == '__main__':
    landbot = LandBotSimulator(real=False)  # Running in cloud mode
    landbot.move_forward(1.0, 2.0)
    landbot.rotate_right(90)
    landbot.move_forward(1.0, 2.0)
    landbot.move_left(0.5, 1.5)
    landbot.rotate_left(45)
    landbot.move_backward(1.0, 2.0)
