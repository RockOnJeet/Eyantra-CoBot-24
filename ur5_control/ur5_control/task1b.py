#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        		===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        		===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ 1605 ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boiler_plate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import tf_transformations
from time import sleep


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''

    ############ Function VARIABLES ############

    # You can remove these variables after reading the instructions. These are just for sample.

    area = -1.0
    width = -1.0

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 
    #	->  Recevice coordiantes from 'detectMarkers' using cv2.aruco library 
    #       and use these coordinates to calculate area and width of aruco detected.
    #	->  Extract values from input set of 4 (x,y) coordinates 
    #       and formulate width and height of aruco detected to return 'area' and 'width'.

    ############################################

    points = np.array(coordinates)  # Coordinates go from top left to bottom right
    # Using Shoelace Formula:
    x = points[:, 0]    # ':' stands for all rows
    y = points[:, 1]
    area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

    # Width
    width_top = np.linalg.norm(points[1] - points[0])
    width_bottom = np.linalg.norm(points[3] - points[2])
    width = (width_top + width_bottom) / 2

    ############################################

    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    # Use this variable as a threshold value to detect aruco markers of certain size.
    # Ex: avoid markers/boxes placed far away from arm's reach position  
    aruco_area_threshold = 1500

    # The camera matrix is defined as per camera info loaded from the plugin used. 
    # You may get this from /camer_info topic when camera is spawned in gazebo.
    # Make sure you verify this matrix once if there are calibration issues.
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

    # The distortion matrix is currently set to 0. 
    # We will be using it during Stage 2 hardware as Intel Realsense Camera provides these camera info.
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    # We are using 150x150 aruco marker size
    size_of_aruco_m = 0.15

    # You can remove these variables after reading the instructions. These are just for sample.
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP : 

    #	->  Convert input BGR image to GRAYSCALE for aruco detection

    #   ->  Use these aruco parameters-
    #       ->  Dictionary: 4x4_50 (4x4 only until 50 aruco IDs)

    #   ->  Detect aruco marker in the image and store 'corners' and 'ids'
    #       ->  HINT: Handle cases for empty markers detection. 

    #   ->  Draw detected marker on the image frame which will be shown later

    #   ->  Loop over each marker ID detected in frame and calculate area using function defined above (calculate_rectangle_area(coordinates))

    #   ->  Remove tags which are far away from arm's reach positon based on some threshold defined

    #   ->  Calculate center points aruco list using math and distance from RGB camera using pose estimation of aruco marker
    #       ->  HINT: You may use numpy for center points and 'estimatePoseSingleMarkers' from cv2 aruco library for pose estimation

    #   ->  Draw frame axes from coordinates received using pose estimation
    #       ->  HINT: You may use 'cv2.drawFrameAxes'

    ############################################

    # print(image.shape)
    while True:
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            break
        except Exception as e:
            print('cvtColor Error! Retrying...')
            sleep(1)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    corners, local_ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # print('Ids: ', local_ids,'\nCorners: ', corners, '\nRejected: ', _)
    # If markers are detected, draw them on the image
    if local_ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids=local_ids)
        for i in range(len(local_ids)):
            area, width = calculate_rectangle_area(corners[i][0])  # Returns area and width of rectangle
            # print(local_ids[i], area, width)
            # If area of the marker is greater than the threshold, add it to the list
            if area > aruco_area_threshold:
                width_aruco_list.append(width)
                ids.append(local_ids[i])
                center_aruco_list.append(np.mean(corners[i][0], axis=0))
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([corners[i][0]], size_of_aruco_m, cam_mat, dist_mat)
                # rvec = rvec.squeeze()
                # rvec = cv2.Rodrigues(rvec)[0]
                # rvec = tf_transformations.euler_from_matrix(rvec)
                # rvec = list(rvec)
                # rvec[0] += np.radians(180)
                # rvec = tf_transformations.euler_matrix(rvec[0], rvec[1], rvec[2])
                # rvec = cv2.Rodrigues(rvec[:3, :3])[0]
                # rvec = rvec.flatten()
                # rvec = np.array([[rvec]])
                # print('id: ', local_ids[i], 'rvec: ', rvec, '\ntvec: ', tvec)
                distance_from_rgb_list.append(tvec[0][0][2])
                angle_aruco_list.append(rvec[0][0][2])
                # Draw frame axes for each marker
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.5, 5)
    
    # Debugging
    # for i in range(len(center_aruco_list)):
    #     print("ID: ", ids[i], "Center: ", center_aruco_list[i], "Distance: ", distance_from_rgb_list[i], "Angle: ", angle_aruco_list[i], "Width: ", width_aruco_list[i])

    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT: You may use CvBridge to do the same

        ############################################

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, '32FC1')

            # Normalize the Image
            norm_depth_img = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            norm_depth_img = np.uint8(norm_depth_img)

            # Debugging
            # cv2.imshow('Depth', norm_depth_img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            # cv2.waitKey(1)
        except Exception as e:
            print('Depth Error: ', e)

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Use data variable to convert ROS Image message to CV2 Image type

        #   ->  HINT:   You may use CvBridge to do the same
        #               Check if you need any rotation or flipping image as input data maybe different than what you expect to be.
        #               You may use cv2 functions such as 'flip' and 'rotate' to do the same

        ############################################

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Debugging
            # cv2.imshow('Color', self.cv_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            # cv2.waitKey(1)
        except Exception as e:
            print('Color Error: ', e)

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''

        ############ Function VARIABLES ############

        # These are the variables defined from camera info topic such as image pixel size, focalX, focalY, etc.
        # Make sure you verify these variable values once. As it may affect your result.
        # You can find more on these variables here -> http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
            

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 

        #	->  Get aruco center, distance from rgb, angle, width and ids list from 'detect_aruco_center' defined above

        #   ->  Loop over detected box ids received to calculate position and orientation transform to publish TF 

        #   ->  Use this equation to correct the input aruco angle received from cv2 aruco function 'estimatePoseSingleMarkers' here
        #       It's a correction formula- 
        #       angle_aruco = (0.788*angle_aruco) - ((angle_aruco**2)/3160)

        #   ->  Then calculate quaternions from roll pitch yaw (where, roll and pitch are 0 while yaw is corrected aruco_angle)

        #   ->  Use center_aruco_list to get realsense depth and log them down. (divide by 1000 to convert mm to m)

        #   ->  Use this formula to rectify x, y, z based on focal length, center value and size of image
        #       x = distance_from_rgb * (sizeCamX - cX - centerCamX) / focalX
        #       y = distance_from_rgb * (sizeCamY - cY - centerCamY) / focalY
        #       z = distance_from_rgb
        #       where, 
        #               cX, and cY from 'center_aruco_list'
        #               distance_from_rgb is depth of object calculated in previous step
        #               sizeCamX, sizeCamY, centerCamX, centerCamY, focalX and focalY are defined above

        #   ->  Now, mark the center points on image frame using cX and cY variables with help of 'cv2.cirle' function 

        #   ->  Here, till now you receive coordinates from camera_link to aruco marker center position. 
        #       So, publish this transform w.r.t. camera_link using Geometry Message - TransformStamped 
        #       so that we will collect it's position w.r.t base_link in next step.
        #       Use the following frame_id-
        #           frame_id = 'camera_link'
        #           child_frame_id = 'cam_<marker_id>'          Ex: cam_20, where 20 is aruco marker ID

        #   ->  Then finally lookup transform between base_link and obj frame to publish the TF
        #       You may use 'lookup_transform' function to pose of obj frame w.r.t base_link 

        #   ->  And now publish TF between object frame and base_link
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = 'obj_<marker_id>'          Ex: obj_20, where 20 is aruco marker ID

        #   ->  At last show cv2 image window having detected markers drawn and center points located using 'cv2.imshow' function.
        #       Refer MD book on portal for sample image -> https://portal.e-yantra.org/

        #   ->  NOTE:   The Z axis of TF should be pointing inside the box (Purpose of this will be known in task 1C)
        #               Also, auto eval script will be judging angular difference as well. So, make sure that Z axis is inside the box (Refer sample images on Portal - MD book)

        ############################################

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)

        for i in range(len(ids)):
            angle_aruco = (0.788*angle_aruco_list[i]) - ((angle_aruco_list[i]**2)/3160)

            q = tf_transformations.quaternion_from_euler(0, 0, angle_aruco)

            # print(center_aruco_list[i])

            distance_from_rgb = self.depth_image[int(center_aruco_list[i][1]), int(center_aruco_list[i][0])] / 1000.0

            x = distance_from_rgb * (sizeCamX - center_aruco_list[i][0] - centerCamX) / focalX
            y = distance_from_rgb * (sizeCamY - center_aruco_list[i][1] - centerCamY) / focalY
            z = distance_from_rgb

            # print('id: ', int(ids[i]), 'x: ', x, 'y: ', y, 'z: ', z)

            cv2.circle(self.cv_image, (int(center_aruco_list[i][0]), int(center_aruco_list[i][1])), 5, (100, 0, 100), -1)

            tempStamped = TransformStamped()
            
            tempStamped.header.stamp = self.get_clock().now().to_msg()
            tempStamped.header.frame_id = 'camera_link'
            tempStamped.child_frame_id = f'cam_{int(ids[i])}'

            tempStamped.transform.translation.x = z
            tempStamped.transform.translation.y = x
            tempStamped.transform.translation.z = y
            tempStamped.transform.rotation.x = q[0]
            tempStamped.transform.rotation.y = q[1]
            tempStamped.transform.rotation.z = q[2]
            tempStamped.transform.rotation.w = q[3]

            self.br.sendTransform(tempStamped)
            
            try:
                trans = self.tf_buffer.lookup_transform('base_link', f'cam_{int(ids[i])}', rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('DEAD')
                continue

            tempStamped = TransformStamped()
            
            tempStamped.header.stamp = self.get_clock().now().to_msg()
            tempStamped.header.frame_id = 'base_link'
            tempStamped.child_frame_id = f'obj_{int(ids[i])}'
            
            tempStamped.transform.translation.x = trans.transform.translation.x
            tempStamped.transform.translation.y = trans.transform.translation.y
            tempStamped.transform.translation.z = trans.transform.translation.z

            q = tf_transformations.quaternion_from_euler(np.pi/2, 0, -angle_aruco + np.pi/2)

            tempStamped.transform.rotation.x = q[0]
            tempStamped.transform.rotation.y = q[1]
            tempStamped.transform.rotation.z = q[2]
            tempStamped.transform.rotation.w = q[3]

            self.br.sendTransform(tempStamped)

        # Debugging
        cv2.imshow('Image', self.cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
                    it sets the special __name__ variable to have a value “__main__”. 
                    If this file is being imported from another module, __name__ will be set to the module’s name.
                    You can find more on this here -> https://www.geeksforgeeks.org/what-does-the-if-__name__-__main__-do/
    '''

    main()
