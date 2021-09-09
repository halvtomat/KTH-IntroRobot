#!/usr/bin/env python3
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Min allowed gain to move along path (in feedback)
min_allowed_gain = 3

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def goal_active():
    pass

def goal_feedback(feedback):
    # Check if this path has higher gain than min_allowed_gain
    if(feedback.gain > min_allowed_gain):
        # If it has cancel goal and move along the path
        goal_client.cancel_all_goals()
        move(feedback.path)

    


def goal_result(state, result):
    # If the state is succeeded then
    if actionlib.TerminalState.SUCCEEDED == state:
        # Move along the path if path is not empty
        if result.path:
            move(result.path)

def move(path):
    global control_client, robot_frame_id, pub

    rate = rospy.Rate(10.0)
    message = Twist()
    # Call service client with path
    while path.poses:
        res = control_client(path)
        setpoint = res.setpoint
        path = res.new_path
        
        # Transform Setpoint from service client
        try:
            transform = tf_buffer.lookup_transform(robot_frame_id, setpoint.header.frame_id, rospy.Time())
            setpoint_new = tf2_geometry_msgs.do_transform_point(setpoint, transform)
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # Create Twist message from the transformed Setpoint
        angle_vel = atan2(setpoint_new.point.y, setpoint_new.point.x)
        message.linear.x = min(setpoint_new.point.x, max_linear_velocity)
        if max_angular_velocity < angle_vel:
            message.linear.x = 0
        message.angular.z = min(angle_vel, max_angular_velocity)

        # Publish Twist
        pub.publish(message)
        rate.sleep()

    # Send 0 control Twist to stop robot
    message.linear.x = 0
    message.angular.z = 0
    pub.publish(message)

    # Get new path from action server
    get_path()

def get_path():
    global goal_client
    # Get path from action server
    goal_client.wait_for_server()
    goal = irob_assignment_1.msg.GetNextGoalActionGoal()
    goal_client.send_goal(goal, done_cb=goal_result, active_cb=goal_active, feedback_cb=goal_feedback)


if __name__ == "__main__":
    # Init node
    rospy.init_node('controller')

    # Init publisher
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Init simple action client
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)

    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint', GetSetpoint)
    
    # Init tf2
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()

    # Spin
    rospy.spin()