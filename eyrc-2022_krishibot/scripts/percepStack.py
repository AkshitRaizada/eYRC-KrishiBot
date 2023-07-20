#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID: KB#1759
# Author List: Akshit Raizada, Aditya Suwalka, Abiroop Mohan, Ebrahim Rampurawala
# Filename: percepStack.py
# Functions:		



####################### IMPORT MODULES #######################
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import roslib
from tf.transformations import euler_from_quaternion
import tf2_ros
import tf_conversions
# You can add more if required
##############################################################


# Initialize Global variables

cX = 0.0
cY = 0.0
depth_image = np.empty((640,480),float)
debug = 1
pose1 = []

################# ADD UTILITY FUNCTIONS HERE #################

##############################################################

def img_clbck1(img_msg):

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    global pub_rgb
    global pose1
    Display_Image = img
    pose = image_processing(img)
    #cv2.putText(Display_Image, "obj0", pose[0], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, ), 2)
    #cv2.putText(Display_Image, "obj1", pose[1], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, ), 2)
    cv2.imwrite("imgseeee1.jpeg" , img)
    
    pose1 = pose
    rospy.loginfo("Pose1 = " + str(pose))
    pub_rgb.publish(str(pose))

def depth_clbck1(depth_msg):
    global pub_depth
    global pose1
    depth_val = []
    depth = 0.0
    ############################### Add your code here #######################################
    bridgeDepth = CvBridge()                #for creating the bridge object
    depth_image = bridgeDepth.imgmsg_to_cv2(depth_msg, "32FC1")
    
    depth_intensity = np.array(5120 * depth_image / 0x0fff, dtype=np.uint8)
    
    cv2.imwrite("Image1.jpeg" , depth_intensity)
    
    fruitname = ["fruit_red" , "fruit_yellow"]
    c = 0
    cx , cy = 320.5 , 240.5
    fx , fy = 554.387 , 554.387
    for pose in pose1:
        cX = pose[0]
        cY = pose[1]
        X = int(cX)
        Y = int(cY)
        rospy.loginfo("Coods1 = " + str((X,Y)))
        depth = depth_image[Y][X] / 1024.0
        depth_val.append(depth)
        
        world_X = depth*((X-cx)/fx)
        world_Y = depth*((Y-cy)/fy)
        world_Z = depth
        
        br = tf2_ros.TransformBroadcaster()     #setting up the TF2 broadcaster
        t = geometry_msgs.msg.TransformStamped()        #broadcasting is stamped for every object
        t.header.stamp = rospy.Time.now()       #the head stamp is the current time that we use this makes it unique
        t.header.frame_id = "camera_link2"          #as the camera on the arm has the camera_link2 so we are using that
        t.child_frame_id = fruitname[c]
        
        
        t.transform.translation.x = world_Z          #this is for transforming the world coordinates to the camera frame that is on the arm
        t.transform.translation.y = -world_X         #this is for transforming the world coordinates to the camera frame that is on the arm
        t.transform.translation.z = -world_Y         #this is for transforming the world coordinates to the camera frame that is on the arm

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)       #for conversion euler to quaternion
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        #br.sendTransform((world_Z, -world_X, -world_Y),
        #                  tf.transformations.quaternion_from_euler(0, 0, 0),
        #                  rospy.Time.now(),
        #                 "base_link or camera_link2",
        #                 fruitname[c])
        c = c + 1
        br.sendTransform(t)
    
    
    ##########################################################################################
    if(not (pose1 == [])):
        rospy.loginfo("Depth1 = " + str(depth_val))
        rospy.loginfo("==================")
        pub_depth.publish(str(depth_val))

def image_processing(image):
    pose = []
    ############### Write Your code to find centroid of the bell peppers #####################
    
    global cX
    global cY
    global debug
    imgHsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    
    h_min_yellow = 0
    h_max_yellow = 30
    s_min_yellow = 110
    s_max_yellow = 255
    v_min_yellow = 118
    v_max_yellow = 255
    
    lower_yellow = np.array([h_min_yellow,s_min_yellow,v_min_yellow])
    upper_yellow = np.array([h_max_yellow,s_max_yellow,v_max_yellow])

    mask_yellow = cv2.inRange(imgHsv,lower_yellow,upper_yellow)
    
    h_min_red = 0
    h_max_red = 10
    s_min_red = 93
    s_max_red = 255
    v_min_red = 47
    v_max_red = 255
    
    lower_red = np.array([h_min_red,s_min_red,v_min_red])
    upper_red = np.array([h_max_red,s_max_red,v_max_red])

    mask_red = cv2.inRange(imgHsv,lower_red,upper_red)
    cv2.imwrite("mask_red.jpeg" , mask_red)
    cv2.imwrite("mask_yellow.jpeg" , mask_yellow)
    mask = mask_yellow | mask_red
    
    cv2.imwrite("mask.jpeg" , mask)
    
    mask2 = cv2.medianBlur(mask, 5)  # Removes noise without affecting edges – but slow

    imgResult2 = cv2.bitwise_and(image, image, mask=mask2)
    imgResult2 = cv2.cvtColor(imgResult2, cv2.COLOR_RGB2GRAY)
    # calculate moments of binary image
    
    maxCnts = 0
    
    cnts, hierarchy = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0:2]
    
    for c in cnt:
        if(cv2.contourArea(c) > 200.0):
            maxCnts = maxCnts + 1
            
        else:
           break
           
    cnt = cnt[0:maxCnts]
    
    for c in cnt:
        M = cv2.moments(c)
        # calculate x,y coordinate of center
        if not M["m00"] == 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            pose.append([cX, cY])

    #to get the HSV --> Hue Saturation Value of the image

    ##########################################################################################
    return pose


def main():
    
    rospy.init_node("percepStack", anonymous=True)
    pub_rgb1 = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_depth1 = rospy.Publisher('/center_depth', String, queue_size = 1)

    global pub_rgb
    global pub_depth
    pub_rgb = pub_rgb1
    pub_depth = pub_depth1
    
    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    
    sub_image_color_1 = rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck1)   #/camera/color/image_raw2
    #rospy.sleep(0.5)
    sub_image_depth_1 = rospy.Subscriber("/camera/depth/image_raw2", Image, depth_clbck1) #/camera/depth/image_raw2
    ####################################################################################################
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")
