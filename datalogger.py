
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tkinter as tk
from tkinter import messagebox, ttk, filedialog
from threading import Timer, Thread
import pandas as pd
import os
from datetime import datetime

class OdomImuLogger(Node):
    def __init__(self, odom_vars, imu_vars, duration, gui_callback, update_progress):
        super().__init__('odom_imu_logger')
        self.odom_vars = odom_vars
        self.imu_vars = imu_vars
        self.duration = duration
        self.gui_callback = gui_callback
        self.update_progress = update_progress
        self.log = []

        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.latest_odom = None
        self.latest_imu = None
        self.start_time = datetime.now()

    def odom_callback(self, msg):
        self.latest_odom = msg
        self.log_combined()

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.log_combined()

    def log_combined(self):
        if self.latest_odom is None or self.latest_imu is None:
            return

        entry = {}
        for var in self.odom_vars:
            try:
                value = self.latest_odom
                for attr in var.split('.'):
                    value = getattr(value, attr)
                entry[var] = value
            except AttributeError:
                entry[var] = None

        for var in self.imu_vars:
            try:
                value = self.latest_imu
                for attr in var.split('.'):
                    value = getattr(value, attr)
                entry[var] = value
            except AttributeError:
                entry[var] = None

        self.log.append(entry)
        self.gui_callback(entry)
        elapsed = (datetime.now() - self.start_time).total_seconds()
        progress_value = min(100, int((elapsed / self.duration) * 100))
        self.update_progress(progress_value)

def browse_folder():
    folder_selected = filedialog.askdirectory()
    if folder_selected:
        folder_entry.delete(0, tk.END)
        folder_entry.insert(0, folder_selected)

def start_logging():
    selected_odom = [var for var, var_state in odom_checkboxes.items() if var_state.get()]
    selected_imu = [var for var, var_state in imu_checkboxes.items() if var_state.get()]
    try:
        duration = int(duration_entry.get())
        if duration <= 0:
            raise ValueError
    except ValueError:
        messagebox.showwarning("Invalid Input", "Please enter a valid number for duration.")
        return

    filename = filename_entry.get().strip()
    if not filename:
        messagebox.showwarning("Missing Filename", "Please enter a file name for the CSV export.")
        return

    directory = folder_entry.get().strip()
    if not directory or not os.path.isdir(directory):
        messagebox.showwarning("Missing Directory", "Please select a valid folder to save the CSV.")
        return

    if not filename.endswith(".csv"):
        filename += ".csv"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    full_path = os.path.join(directory, f"{timestamp}_{filename}")

    if not selected_odom and not selected_imu:
        messagebox.showwarning("No selection", "Please select at least one variable to log.")
        return

    for col in tree.get_children():
        tree.delete(col)
    progress_bar["value"] = 0

    def update_progress_bar(val):
        progress_bar.after(0, lambda val=val: progress_bar.configure(value=val))

    def run_logger():
        rclpy.init()
        logger_node = OdomImuLogger(
            selected_odom,
            selected_imu,
            duration,
            display_log_entry,
            update_progress_bar
        )
        rclpy_thread = Thread(target=rclpy.spin, args=(logger_node,), daemon=True)
        rclpy_thread.start()

        def stop_and_exit():
            rclpy.shutdown()
            df = pd.DataFrame(logger_node.log)
            df.to_csv(full_path, index=False)
            messagebox.showinfo("Finished", f"Logging finished. Data saved to '{full_path}'.")

        Timer(duration, stop_and_exit).start()

    Thread(target=run_logger).start()

def display_log_entry(entry):
    if not tree["columns"]:
        columns = list(entry.keys())
        tree["columns"] = columns
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, width=100, anchor="center")
    values = [f"{entry[k]:.3f}" if entry[k] is not None else "N/A" for k in tree["columns"]]
    tree.insert("", "end", values=values)

# GUI Setup
root = tk.Tk()
root.title("ROS2 Odometry + IMU Logger")

tk.Label(root, text="Select odometry variables to log:").pack()
odom_checkboxes = {}
AVAILABLE_ODOM_VARS = [
    ('pose.pose.position.x', 'm'),
    ('pose.pose.position.y', 'm'),
    ('pose.pose.position.z', 'm'),
    ('twist.twist.linear.x', 'm/s'),
    ('twist.twist.angular.z', 'rad/s')
]
for var, unit in AVAILABLE_ODOM_VARS:
    var_state = tk.BooleanVar()
    odom_checkboxes[var] = var_state
    tk.Checkbutton(root, text=f"{var} ({unit})", variable=var_state).pack(anchor='w')

tk.Label(root, text="Select IMU variables to log:").pack()
imu_checkboxes = {}
AVAILABLE_IMU_VARS = [
    ('angular_velocity.x', 'rad/s'),
    ('angular_velocity.y', 'rad/s'),
    ('angular_velocity.z', 'rad/s'),
    ('linear_acceleration.x', 'm/s²'),
    ('linear_acceleration.y', 'm/s²'),
    ('linear_acceleration.z', 'm/s²')
]
for var, unit in AVAILABLE_IMU_VARS:
    var_state = tk.BooleanVar()
    imu_checkboxes[var] = var_state
    tk.Checkbutton(root, text=f"{var} ({unit})", variable=var_state).pack(anchor='w')

tk.Label(root, text="Enter logging duration (seconds):").pack()
duration_entry = tk.Entry(root)
duration_entry.insert(0, "5")
duration_entry.pack()

tk.Label(root, text="Enter CSV file name:").pack()
filename_entry = tk.Entry(root)
filename_entry.insert(0, "odom_imu_log.csv")
filename_entry.pack()

tk.Label(root, text="Select folder to save file:").pack()
folder_frame = tk.Frame(root)
folder_frame.pack(fill="x")
folder_entry = tk.Entry(folder_frame)
folder_entry.pack(side="left", fill="x", expand=True)
tk.Button(folder_frame, text="Browse", command=browse_folder).pack(side="right")

tk.Button(root, text="Start Logging", command=start_logging).pack(pady=5)

progress_bar = ttk.Progressbar(root, orient="horizontal", length=300, mode="determinate")
progress_bar.pack()

tree = ttk.Treeview(root, show='headings')
tree.pack(expand=True, fill='both')

root.mainloop()
