# import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class OdometryPlotter:
    def __init__(self, root):
        self.root = root
        self.root.title("Odometry Acceleration Plot")

        # Create a figure and axis for the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Odometry Acceleration Over Time')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Acceleration (m/s^2)')

        # Initialize the data for plotting
        self.time_data = []
        self.accel_data = []

        # Create a Matplotlib canvas to embed in the Tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=1)

        # Subscribe to the ROS topic
        rospy.Subscriber('/artcar_1/odometry/filtered', Odometry, self.odometry_callback)

        # Start the animation for the plot
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

    def odometry_callback(self, data):
        # Extract acceleration from the odometry message
        accel = data.twist.twist.linear  # Assuming the acceleration is in the linear twist
        acceleration = np.sqrt(accel.x**2 + accel.y**2 + accel.z**2)  # Magnitude of acceleration

        # Append the current time and acceleration to the data
        self.time_data.append(rospy.get_time())
        self.accel_data.append(acceleration)

        # Limit the size of the data to keep the plot responsive
        if len(self.time_data) > 100:
            self.time_data.pop(0)
            self.accel_data.pop(0)

    def update_plot(self, i):
        # Clear the current plot
        self.ax.clear()

        # Plot the updated data
        self.ax.plot(self.time_data, self.accel_data, label='Acceleration (m/s^2)')
        self.ax.legend()

        # Redraw the canvas
        self.canvas.draw()

if __name__ == "__main__":
    # Initialize the Tkinter application
    root = tk.Tk()

    # Initialize the ROS node
    rospy.init_node('odometry_plotter', anonymous=True)

    # Create an instance of the plotter
    plotter = OdometryPlotter(root)

    # Start the Tkinter main loop
    root.mainloop()
