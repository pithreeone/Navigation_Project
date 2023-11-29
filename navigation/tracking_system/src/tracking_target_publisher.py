#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk

class TrackingTargetPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tracking_target_publisher', anonymous=True)

        # Create a publisher for the /tracking_target topic
        self.publisher = rospy.Publisher('tracking_target', Pose, queue_size=10)

        # Set up the plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.scatter = self.ax.scatter([], [])
        self.canvas = FigureCanvasTkAgg(self.fig, master=tk.Tk())
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.canvas.draw()

        # Connect the mouse click event to the handler
        self.canvas.mpl_connect('button_press_event', self.on_mouse_click)

    def on_mouse_click(self, event):
        # Get the mouse position and publish it
        mouse_x, mouse_y = event.xdata, event.ydata
        if mouse_x is not None and mouse_y is not None:
            pose_message = Pose()
            pose_message.position.x = mouse_x
            pose_message.position.y = mouse_y
            pose_message.position.z = 0.0
            self.publisher.publish(pose_message)
            rospy.loginfo("Published /tracking_target message at ({}, {})".format(mouse_x, mouse_y))

            # Update the plot with the clicked point
            self.scatter.set_offsets([[mouse_x, mouse_y]])
            self.canvas.draw()

if __name__ == '__main__':
    try:
        tracking_target_publisher = TrackingTargetPublisher()

        # Run the Tkinter main loop
        tk.mainloop()
    except rospy.ROSInterruptException:
        pass
