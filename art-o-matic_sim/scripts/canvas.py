#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import std_msgs.msg as std
import geometry_msgs.msg as geo

#global variables
NO_OF_ROBOTS = 10 #number of robots
drawing = False #true if mouse is pressed
x_prev, y_prev = -1, -1 #previous mouse coordinates
target = [] #list of target pixels
target_publisher = [] #publisher for target positions

# Mouse callback function
def draw(event, x, y, flags, param):
    global drawing, x_prev, y_prev

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            cv2.line(canvas, (x_prev,y_prev), (x, y), (255, 255, 255), 2)
        x_prev, y_prev = x, y
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

# Function to decide the robots' target positions
def get_target_pose():
    global target
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(canvas, contours, -1, (0, 255, 0), 3)
    contours = contours[0].reshape(-1, 2)
    target = []
    ind =  int(contours.shape[0]/10)
    for i in range(0, contours.shape[0], ind):
        target.append(contours[i])
        cv2.circle(canvas, tuple(contours[i]), 2, (0, 0, 255), 2)
    publish_msg()

# Publish target positions
def publish_msg():
    global target, target_publisher
    for i in range(0, NO_OF_ROBOTS):
        target_publisher[i].publish(geo.Point(target[i][0]/500, target[i][1]/500, 0))

#main function
def main():
    global canvas, target_publisher, NO_OF_ROBOTS

    print("Number of robots: ", NO_OF_ROBOTS)
    print("Draw the shape in the canvas")
    print("Press 'm' to send image to the artist")
    print("Press 'c' to clear the canvas")
    print("Press 'q' to quit")

    # Initializing ROS node
    rospy.init_node('swarm_controller')
    for i in range(0,NO_OF_ROBOTS):
        target_publisher.append(rospy.Publisher('/artist'+str(i)+'/target', geo.Point, queue_size=10))

    # Creating a black image, a window and bind the function to window
    canvas = np.zeros((500, 500, 3), np.uint8)
    cv2.namedWindow('canvas')
    cv2.setMouseCallback('canvas', draw)

    while not rospy.is_shutdown():
        cv2.imshow('canvas', canvas)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('m'):
            get_target_pose()
        elif k == ord('c'):
            canvas = np.zeros((500, 500, 3), np.uint8)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass