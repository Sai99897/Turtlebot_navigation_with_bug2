#  Turtlebot_navigation_with_bug2

This repository contains the codes to reach all the published goals by moving the robot in a gazebo environment by avoiding all the obstacles in the environment.

**How to start:**

There are two main task to be accomplished in this project by the robot:

  1.To reach all the goals published by /goals topic from goal_publisher package.  
  2.To avoid the obstacles while reaching the goals by the algorithm.
  
I make use BUG2 algorithm technique for achieving this task.

The prerequisites include:

  1.Having a basic knowledge on the ROS topics ,Subscribers and Publishers.  
  2.Creating a Gazebo environment with the obstacles.
  
**General Description:**
The aim of the project is avoid all the obstacles and reach as many goals as possible.

The topics being used are:
1. /cmd_vel : This is used to provide motion for the robot. The robot moves is x axis and rotates along the z axis.
2. /gazebo/model_states : For getting the information of current position of the robot in the environment.
3. /goals : This is used to get the goals data and preprocess it for reaching all the goals.
4. /scan : This is used to get the Laser scan information.

**Algorithm Description :**

The main aim is to reach all the goals by avoiding all the obstacles.This is accomplished using the BUG2 algorithm:

This project implements a wall-following algorithm in python for an autonomous mobile 2 wheeled robot with a laser scanner sensor using the Robot operating system(ROS)libraries,
Gazebo as simulator and Python as programming language. The proposed wall-following algorithm makes a robot wander at random until a wall is found, then follows the wall - through
an implemented proportional control to keep a constant distance from it. In this algorithm we are not driving the robot in a straight line to the desired point, but following the
original line. I am using statemachine concept to shift the state based on the logic. In go to point program yaw is found to turn the robot to the destination and in follow wall
program if any obstacle comes it turns left and make sure that wall is on the left side of the robot. In this project I made use of services.

**Problems:**

To implement this project first I was stuck in reading the laser data finally I accomplished it by reading some blogs and youtube videos.


