#!/usr/bin/env python3

import rospy

import kinova_msgs
import actionlib

from sensor_msgs.msg import Joy
from kinova_msgs.msg import PoseVelocity

from kinova_msgs.srv import HomeArm


class XboxVelocityController:
    """ Xbox controller with the following code.

    | **Index** | **Button Name on the Actual Controller** |
    |-----------|-----------------------------------------|
    | 0         | A                                       |
    | 1         | B                                       |
    | 2         | X                                       |
    | 3         | Y                                       |
    | 4         | LB                                      |
    | 5         | RB                                      |
    | 6         | Back                                    |
    | 7         | Start                                   |
    | 8         | Power                                   |
    | 9         | Button Stick Left                       |
    | 10        | Button Stick Right                      |


    | **Index** | **Axis Name on the Actual Controller** |
    |-----------|----------------------------------------|
    | 0         | Left/Right Axis Stick Left            |
    | 1         | Up/Down Axis Stick Left               |
    | 2         | Left/Right Axis Stick Right           |
    | 3         | Up/Down Axis Stick Right              |
    | 4         | RT                                    |
    | 5         | LT                                    |
    | 6         | Cross Key Left/Right                  |
    | 7         | Cross Key Up/Down                     |

    """
    def __init__(self, fingers_closed_pos: int = 5000, ros_rate: int = 100):
        """Initialize a xbox controller class.

        Args:
            fingers_closed_pos (int, optional): Close command send to the gripper,
                increase for harder grasp. Defaults to 5000.
            ros_rate (int, optional): ROS update rate. Defaults to 100.
        """
        rospy.init_node('xbox_velocity_controller', anonymous=True)

        # robot end-effector velocity publisher
        self.velocity_pub = rospy.Publisher('/j2s7s300_driver/in/cartesian_velocity',
                                            PoseVelocity, queue_size=10)

        # xbox controller subscriber
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        # service to send the arm home
        rospy.wait_for_service('/j2s7s300_driver/in/home_arm')
        self.home_arm_service = rospy.ServiceProxy('/j2s7s300_driver/in/home_arm', HomeArm)

        # initial twist values
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        self.rate = rospy.Rate(ros_rate)
        self.fingers_closed_pos = fingers_closed_pos
        self.joy_info_available: bool = False

    def joy_callback(self, msg: Joy):
        """Callback function on xbox controller topic.

        Args:
            msg (Joy): Joy message received from controller.
        """

        # x/y linear velocity
        self.linear_x = msg.axes[0]
        self.linear_y = msg.axes[1]

        # z linear velocity, up and down
        lb_pressed = msg.buttons[4]
        rb_pressed = msg.buttons[5]

        if lb_pressed and not rb_pressed:
            self.linear_z = 1.0
        elif rb_pressed and not lb_pressed:
            self.linear_z = -1.0
        else:
            self.linear_z = 0.0

        # gripper action
        x_pressed = msg.buttons[2]
        y_pressed = msg.buttons[3]

        if x_pressed and not y_pressed:
            # close the gripper
            self.gripper_client([self.fingers_closed_pos] * 3)
        elif y_pressed and not x_pressed:
            # open the gripper
            self.gripper_client([0] * 3)
        else:
            pass

        # home position
        back_pressed = msg.buttons[6]

        if back_pressed:
            print('Right stick is pressed!')
            self.home_arm_service()
            print('Service is executed!')


        # x/y angular velocity
        self.angular_x = msg.axes[4]
        self.angular_y = msg.axes[3]

        # z angular velocity, gripper rotation
        a_pressed = msg.buttons[0]
        b_pressed = msg.buttons[1]

        if a_pressed and not b_pressed:
            self.angular_z = 1.0
        elif a_pressed and not b_pressed:
            self.angular_z = -1.0
        else:
            self.angular_z = 0.0

    def gripper_client(self, finger_positions):
        """Gripper client sends finger positions to open and close the gripper.

        Args:
            finger_positions (ListLike): List of positions of three fingers.

        """
        action_address = '/j2s7s300_driver/fingers_action/finger_positions'

        client = actionlib.SimpleActionClient(action_address,
                                              kinova_msgs.msg.SetFingersPositionAction)
        client.wait_for_server()

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions[0])
        goal.fingers.finger2 = float(finger_positions[1])

        if len(finger_positions) < 3:
            goal.fingers.finger3 = 0.0
        else:
            goal.fingers.finger3 = float(finger_positions[2])

        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(5.0)):
            pass
        else:
            client.cancel_all_goals()
            print('Failed action cancelling goals!')

    def run(self):
        """Main loop to send twist commands based on xbox controller readings.
        """
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

# main entry
if __name__ == '__main__':

    try:
        controller = XboxVelocityController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
