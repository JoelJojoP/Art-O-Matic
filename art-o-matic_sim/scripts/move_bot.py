#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt
from geometry_msgs.msg import Point
import sys

robot_number = 0 # Robot's number
robot_position= None # Robot's current position
robot_position_theta = None # Robot's current orientation
other_robot_positions = {} # Other robots' positions
target_position = None # Target position

velocity_pub = None # Publisher for the robot's velocity

def robot_odom_callback(msg):
    global robot_position, robot_position_theta

    # Update the robot's current position
    robot_position = msg.pose.pose.position
    robot_position_theta = msg.pose.pose.orientation.z

# def other_robot_odom_callback(msg):
#     global other_robot_positions

#     # Update the position of other robots
#     robot_name = msg._connection_header['topic'].split('/')[1]  # Extract the robot name from the topic
#     other_robot_positions[robot_name] = msg.pose.pose.position

def target_callback(msg):
    global target_position

    # Update the target position
    target_position = msg

def calculate_dis(x1,y1,x2,y2):
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))

def main():
    global velocity_pub, robot_position, robot_position_theta, target_position, other_robot_positions, robot_number

    # Initialize ROS node
    rospy.init_node('move_bot', anonymous=True)

    while not rospy.is_shutdown():
        # Subscribe to robot's own odometry topic
        robot_odom_sub = rospy.Subscriber('/artist'+str(robot_number)+'/odom', Odometry, robot_odom_callback)

        # Create a publisher for the robot's velocity
        velocity_pub = rospy.Publisher('/artist'+str(robot_number)+'/cmd_vel', Twist, queue_size=10)

        # Subscribe to taget position topic
        target_sub = rospy.Subscriber('/artist'+str(robot_number)+'/target', Point, target_callback)

        # Subscribe to other robots' odom topic
        # other_robot_odom = []
        # for i in range(0, 10):
        #     if i is not robot_number:
        #         other_robot_odom.append(rospy.Subscriber('/artist'+str(i)+'/odom', Odometry, other_robot_odom_callback))

        rate = rospy.Rate(10)  # 10 Hz
        cmd_vel = Twist()

        while not rospy.is_shutdown():
            if robot_position is not None and target_position is not None:
                rospy.loginfo("Robot %d is moving to the target position %f %f. Current Pose: %f %f %f", robot_number, target_position.x, target_position.y, robot_position.x, robot_position.y, robot_position_theta)
                # Calculate the distance to the target position
                distance = calculate_dis(robot_position.x, robot_position.y,target_position.x, target_position.y)

                if distance > 0.05:  # If the robot is not at the target position
                    # Calculate the linear and angular velocities for navigation
                    linear_velocity = 0.5
                    angular_velocity = 4 * (atan2(target_position.y - robot_position.y,target_position.x - robot_position.x) - robot_position_theta)

                    # Adjust the angular velocity to avoid other robots
                    # for robot_pose in other_robot_positions.values():
                    #     robot_distance = calculate_dis(robot_position.x,robot_position.y,robot_pose.x, robot_pose.y)
                    #     if robot_distance < 0.05:  # Adjust the angular velocity if another robot is too close
                    #         angular_velocity += 1.5 * (atan2(robot_pose.y - robot_position.y,robot_pose.x - robot_position.x) - robot_position_theta)

                    # Set the linear and angular velocities
                    cmd_vel.linear.x = linear_velocity
                    cmd_vel.angular.z = angular_velocity

                    # Publish the velocity command
                    velocity_pub.publish(cmd_vel)
                else:
                    # Stop the robot when it reaches the target position
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    velocity_pub.publish(cmd_vel)
            rate.sleep()
        rospy.loginfo("Robot %d has reached the target position", robot_number)
        if(robot_number<9):
            robot_number+=1
        else:
            robot_number=0
        target_position = None
        robot_position = None
    
if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
       pass 
