#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from goal_publisher.msg import PointArray
import math
from std_srvs.srv import *

import math
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
goal_ =PointArray()
position_ = Point()
initial_position_ = Point()
initial_position_.x = 0.001
initial_position_.y = 0.001
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = 1.5
desired_position_.y = 0
desired_position_.z = 0
distance_diff = 0.5
k = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose[1].position

    # yaw
    quaternion = (
        msg.pose[1].orientation.x,
        msg.pose[1].orientation.y,
        msg.pose[1].orientation.z,
        msg.pose[1].orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[270:300]), 10),
        'fright': min(min(msg.ranges[301:329]), 10),
        'front':  min(min(msg.ranges[0:8] + msg.ranges[330:360]), 10),
        'fleft':  min(min(msg.ranges[10:40]), 10),
        'left':   min(min(msg.ranges[40:70]), 10),
    }

def clbk_point(data):

    global desired_position_,goal_,k
    l = len(data.goals)

    if k < 1:
        goal_x = data.goals[0].x
        goal_y = data.goals[0].y
    else:
        if k < l:
            goal_x = data.goals[k].x
            goal_y = data.goals[k].y

    if k < l:
        if(abs(goal_x - position_.x)>distance_diff and abs(goal_y - position_.y)>distance_diff):
            desired_position_.x = goal_x
            desired_position_.y = goal_y
            initial_position_.x = position_.x
            initial_position_.y = position_.y
            print("Towards Goal: {}".format(k+1))
        elif(abs(goal_x - position_.x)<distance_diff and abs(goal_y - position_.y)<distance_diff):
            k += 1
            print("Goal {} Completed".format(k))
            if k < l:
                desired_position_.x = data.goals[k].x
                desired_position_.y = data.goals[k].y
                initial_position_.x = position_.x
                initial_position_.y = position_.y
        else:
            print("Going towards the goal: {}".format(k+1))


def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_
    # here goes the equation
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():

    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_

    rospy.init_node('bug2')

    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_go = rospy.Subscriber('/goals', PointArray, clbk_point)
    sub_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, clbk_odom)

    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')

    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)

    # initialize going to the point
    change_state(0)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        distance_position_to_line = distance_to_line(position_)

        if state_ == 0:
            if regions_['front'] > 0.15 and regions_['front'] < 1:
                change_state(1)

        elif state_ == 1:
            if count_state_time_ > 5 and \
               distance_position_to_line < 0.1:
                change_state(0)

        count_loop_ = count_loop_ + 1
        if count_loop_ == 5:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
        rate.sleep()

if __name__ == "__main__":
    main()
