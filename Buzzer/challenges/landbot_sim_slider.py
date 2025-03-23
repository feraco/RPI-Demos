
import json
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from ipywidgets import interact
import math

class LandBotSimulator():
    def __init__(self, real=False):
        self.real_mode = real
        self._init_state()
        print("LandBot Simulator with Slider Visualization Initialized")

    def _init_state(self):
        self.position = (0, 0, 0)
        self.bearing = 0
        self.path_coors = [(0, 0, 0)]
        self.yaw_data = [0]
        self.command_log = []

    def move(self, vx, vy, duration):
        vz = 0.1  # add vertical motion for 3D effect
        new_x = self.position[0] + vx * duration
        new_y = self.position[1] + vy * duration
        new_z = self.position[2] + vz
        self.position = (new_x, new_y, new_z)
        self.path_coors.append((new_x, new_y, new_z))

    def move_forward(self, speed, duration):
        self.send_command('move_forward', speed, duration)
        self.move(speed * np.cos(np.radians(self.bearing)), speed * np.sin(np.radians(self.bearing)), duration)

    def move_backward(self, speed, duration):
        self.send_command('move_backward', speed, duration)
        self.move(-speed * np.cos(np.radians(self.bearing)), -speed * np.sin(np.radians(self.bearing)), duration)

    def move_left(self, speed, duration):
        self.send_command('move_left', speed, duration)
        self.move(-speed, 0, duration)

    def move_right(self, speed, duration):
        self.send_command('move_right', speed, duration)
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

    def stop(self):
        self.command_log.append("STOP")
        print("🛑 Robot stopped.")

    def send_command(self, command: str, *args):
        command_json = {'command': command, 'arguments': args}
        self.command_log.append(command_json)
        print(f'Executing command: {LandBotSimulator.serialize_command(command_json)}')
        time.sleep(0.2)

    @staticmethod
    def serialize_command(command: dict):
        serialized = command['command']
        if command.get('arguments'):
            serialized = f"{serialized} {' '.join(map(str, command['arguments']))}"
        return serialized

    def slider_visualization(self):
        coords = np.array(self.path_coors)
        x, y, z = coords[:, 0], coords[:, 1], coords[:, 2]

        @interact(step=(1, len(x)))
        def update(step=1):
            plt.figure(figsize=(6, 4))
            ax = plt.axes(projection='3d')
            ax.plot3D(x[:step], y[:step], z[:step], 'blue')
            ax.scatter(x[0], y[0], z[0], color='green', label='Start')
            ax.scatter(x[step-1], y[step-1], z[step-1], color='red', label='Current')
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            ax.set_title('LandBot Movement with Slider')
            plt.legend()
            plt.show()


    def animate_2d_path(self):
        coords = np.array(self.path_coors)
        x, y = coords[:, 0], coords[:, 1]

        from matplotlib import pyplot as plt
        from matplotlib import animation
        from IPython.display import HTML

        fig, ax = plt.subplots()
        line, = ax.plot([], [], lw=2)
        ax.set_xlim((min(x)-1, max(x)+1))
        ax.set_ylim((min(y)-1, max(y)+1))
        ax.set_title("LandBot 2D Path Animation")
        ax.set_xlabel("X"); ax.set_ylabel("Y")

        def init():
            line.set_data([], [])
            return (line,)

        def animate(i):
            line.set_data(x[:i], y[:i])
            return (line,)

        ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(x)+1, interval=300, blit=True)
        return HTML(ani.to_jshtml())
