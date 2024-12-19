#!/usr/bin/env python3
#
# import rospy
# from sensor_msgs.msg import Joy
# import enum
#
# def joy_callback(msg):
#     # Map Axes
#     axes_mapping = {
#         0: "Left Stick Horizontal",
#         1: "Left Stick Vertical",
#         2: "Left Trigger",
#         3: "Right Stick Horizontal",
#         4: "Right Stick Vertical",
#         5: "Right Trigger",
#         6: "D-Pad Horizontal",
#         7: "D-Pad Vertical"
#     }
#
#     # Map Buttons
#     buttons_mapping = {
#         0: "A",
#         1: "B",
#         2: "X",
#         3: "Y",
#         4: "Left Bumper",
#         5: "Right Bumper",
#         6: "Back",
#         7: "Start",
#         8: "Xbox Button",
#         9: "Left Stick Click",
#         10: "Right Stick Click"
#     }
#
#     # Print Axes with Descriptions
#     rospy.loginfo("Axes:")
#     for i, value in enumerate(msg.axes):
#         axis_name = axes_mapping.get(i, f"Unknown Axis {i}")
#         rospy.loginfo(f"  {axis_name}: {value}")
#
#     # Print Buttons with Descriptions
#     rospy.loginfo("Buttons:")
#     for i, value in enumerate(msg.buttons):
#         button_name = buttons_mapping.get(i, f"Unknown Button {i}")
#         rospy.loginfo(f"  {button_name}: {'Pressed' if value else 'Released'}")
#
# def xbox_reader():
#     rospy.init_node('xbox_reader', anonymous=True)
#     rospy.Subscriber('/joy', Joy, joy_callback)
#     rospy.loginfo("Xbox controller reader started. Listening on /joy...")
#     rospy.spin()
#
# if __name__ == '__main__':
#     try:
#         xbox_reader()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python3


#!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import Joy
# from kinova_msgs.msg import PoseVelocity  # Import for Cartesian velocity messages

# # Scaling factor for very slow velocity (m/s)
# SLOW_VELOCITY_SCALE = 20.0  # Adjusted for even slower movement

# class XboxVelocityController:
#     def __init__(self):
#         # Initialize the ROS node
#         rospy.init_node('xbox_velocity_controller', anonymous=True)

#         # Initialize the publisher for Cartesian velocity
#         self.velocity_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', PoseVelocity, queue_size=10)

#         # Initialize the subscriber for the Xbox controller
#         rospy.Subscriber('/joy', Joy, self.joy_callback)

#         # Initialize the last known velocities
#         self.linear_x = 0.0
#         self.linear_y = 0.0
#         self.linear_z = 0.0
#         self.angular_x = 0.0
#         self.angular_y = 0.0
#         self.angular_z = 0.0

#         # Set the rate to 100 Hz
#         self.rate = rospy.Rate(100)  # 100 Hz

#         rospy.loginfo("Xbox velocity controller started. Listening on /joy...")

#     def joy_callback(self, msg):
#         # Update velocities based on joystick input
#         # Left stick horizontal controls linear x
#         self.linear_x = msg.axes[0] * SLOW_VELOCITY_SCALE
#         # Left stick vertical controls linear y
#         self.linear_y = msg.axes[1] * SLOW_VELOCITY_SCALE

#         # Right stick horizontal controls angular z (rotation around z-axis)
#         self.angular_z = msg.axes[3] * SLOW_VELOCITY_SCALE

#         # Left and Right Triggers control linear z (up/down)
#         # Assuming triggers range from -1 (released) to 1 (fully pressed)
#         # Adjust accordingly if your controller's trigger values differ
#         left_trigger = (1 - msg.axes[2]) / 2  # Normalize to 0 (released) to 1 (pressed)
#         right_trigger = (1 - msg.axes[5]) / 2
#         self.linear_z = (right_trigger - left_trigger) * SLOW_VELOCITY_SCALE

#     def run(self):
#         while not rospy.is_shutdown():
#             # Create a PoseVelocity message
#             velocity_msg = PoseVelocity()
#             velocity_msg.twist_linear_x = self.linear_x
#             velocity_msg.twist_linear_y = self.linear_y
#             velocity_msg.twist_linear_z = self.linear_z
#             velocity_msg.twist_angular_x = self.angular_x
#             velocity_msg.twist_angular_y = self.angular_y
#             velocity_msg.twist_angular_z = self.angular_z

#             # Publish the velocity message
#             self.velocity_pub.publish(velocity_msg)

#             # Sleep to maintain 100 Hz rate
#             self.rate.sleep()

# if __name__ == '__main__':
#     try:



#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from kinova_msgs.msg import PoseVelocity

SLOW_VELOCITY_SCALE = 20.0

class XboxVelocityController:
    def __init__(self):
        rospy.init_node('xbox_velocity_controller', anonymous=True)
        self.velocity_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity', PoseVelocity, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0
        self.rate = rospy.Rate(100)

    def joy_callback(self, msg):
        self.linear_x = msg.axes[0] * SLOW_VELOCITY_SCALE
        self.linear_y = msg.axes[1] * SLOW_VELOCITY_SCALE
        lb_pressed = msg.buttons[4]
        rb_pressed = msg.buttons[5]
        if lb_pressed and not rb_pressed:
            self.linear_z = SLOW_VELOCITY_SCALE
        elif rb_pressed and not lb_pressed:
            self.linear_z = -SLOW_VELOCITY_SCALE
        else:
            self.linear_z = 0.0
        self.angular_x = msg.axes[4] * SLOW_VELOCITY_SCALE
        self.angular_y = msg.axes[3] * SLOW_VELOCITY_SCALE
        self.angular_z = msg.axes[3] * SLOW_VELOCITY_SCALE

    def run(self):
        while not rospy.is_shutdown():
            velocity_msg = PoseVelocity()
            velocity_msg.twist_linear_x = self.linear_x
            velocity_msg.twist_linear_y = self.linear_y
            velocity_msg.twist_linear_z = self.linear_z
            velocity_msg.twist_angular_x = self.angular_x
            velocity_msg.twist_angular_y = self.angular_y
            velocity_msg.twist_angular_z = self.angular_z
            self.velocity_pub.publish(velocity_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = XboxVelocityController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

        controller = XboxVelocityController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
