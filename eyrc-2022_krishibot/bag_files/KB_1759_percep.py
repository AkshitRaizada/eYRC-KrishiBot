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

  


import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import cv2
import numpy as np
import roslib
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import actionlib
import math


##############################################################

cX = 0.0
cY = 0.0
depth_image = np.empty((480,848),float)
debug = 1
pose1 = []
depth_fin = []
##############################################################

def img_clbck(img_msg):

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    global pub_rgb
    global pose1
    
    pose = image_processing(img)
    pose1 = pose
    rospy.loginfo("Pose1 = " + str(pose))
    pub_rgb.publish(str(pose))
    
def depth_clbck(depth_msg):
    global pub_depth
    global pose1
    global depth_fin
    depth_val = []
    depth = 0.0
    bridgeDepth = CvBridge()                #for creating the bridge object
    depth_image = bridgeDepth.imgmsg_to_cv2(depth_msg, "32FC1")
    
    for pose in pose1:
        cX = pose[0]
        cY = pose[1]
        X = int(cX / (720.0/480.0))
        Y = int(cY / (1280.0/848.0))
        depth = depth_image[Y][X]
        depth_val.append(depth)

    if(not (pose1 == [])):
        rospy.loginfo("Depth1 = " + str(depth_val))
        rospy.loginfo("==================")
        depth_fin = depth_val
        pub_depth.publish(str(depth_val))
        
            for counter in range(2)    # 2 fruits
                
                world_X = depth_fin[counter]*((X-cx)/fx)
                world_Y = depth_fin[counter]*((Y-cy)/fy)
                world_Z = depth_fin[counter]
                
                br = tf2_ros.TransformBroadcaster()     #setting up the TF2 broadcaster
                t = geometry_msgs.msg.TransformStamped()        #broadcasting is stamped for every object
                t.header.stamp = rospy.Time.now()       #the head stamp is the current time that we use this makes it unique
                t.header.frame_id = "camera_link2"          #as the camera on the arm has the camera_link2 so we are using that
                t.child_frame_id = "obj"+str(counter)           #this is the naming convention where the is given as obj + value of the counter -----> obj1, obj2 etc.

                cv2.putText(imgContour, t.child_frame_id,               #this function is used to put text on the imgContour, the text is the child_frame_id, at the point (midX, midY) 
                (world_X, world_Y), cv2.FONT_HERSHEY_SIMPLEX,         #cv2.FONT_HERSHEY_SIMPLEX ----> is the font used to label the image
                0.5, (255, 0, ), 2)             #0.5 is the font scale, (255, 0, 0) is for giving blue colour and 2 is the thickness


                ''' In this section we are trying to get the depth of the tomato visible ---> by doing this we will ensure that we accuate the arm only when the depth of the visible tomato
                is upto a certain limit so that we are unnecessarily don't try to pick up tomatoes on the other side of the plant and in doing so hit the plant and damage the arm'''    

                t.transform.translation.x = world_Z          #this is for transforming the world coordinates to the camera frame that is on the arm
                t.transform.translation.y = -world_X         #this is for transforming the world coordinates to the camera frame that is on the arm
                t.transform.translation.z = -world_Y         #this is for transforming the world coordinates to the camera frame that is on the arm

                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)       #for conversion euler to quaternion
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                br.sendTransform(t)         #for broadcating the TF 

                      
def image_processing(image):

    pose = []
    
    cv2.imshow(image)         #If you're getting an error here then convert to cv2 format
    
    global cX
    global cY
    global debug
    imgHsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    
    h_min = 0
    h_max = 180
    s_min = 140
    s_max = 255
    v_min = 100
    v_max = 255
    
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    mask = cv2.inRange(imgHsv,lower,upper)
    mask2 = cv2.medianBlur(mask, 5)  # Removes noise without affecting edges – but slow

    imgResult2 = cv2.bitwise_and(image, image, mask=mask2)
    imgResult2 = cv2.cvtColor(imgResult2, cv2.COLOR_RGB2GRAY)
    # calculate moments of binary image
    
    maxCnts = 0
    
    cnts, hierarchy = cv2.findContours(mask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0:2]
    
    for c in cnt:
        if(cv2.contourArea(c) > 4000.0):
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
    return pose

class Ur5Moveit:

    # Constructor
    def __init__(self):

        self._planning_group = "arm"                                        #for defining arm as the planning group as arm for moving the arm
        self._commander = moveit_commander.roscpp_initialize(sys.argv)      #initializing the moveit commander
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)     #setting the planning group
        self._hand_group = moveit_commander.MoveGroupCommander("gripper")  #this is to initialize a new group called the _hand_group so that the opening and closing of the gripper can be controlled
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()


    def gripper_robot(self,state):                      #this function is used to control the gripper and it accepts the state variable for setting the state of the gripper
        if state == 1:                                  #state 1 means that the gripper will be in the closed state
            self._hand_group.set_named_target("close")  #this is to set the target to a pre-defined pose of the gripper as close
            plan2 = self._hand_group.go()               #this line plans and executes the instructions given by moveit
        elif state == 0:                                #state 0 means that the gripper will be in the open state
            self._hand_group.set_named_target("open")   #this sets the state to open
            plan2 = self._hand_group.go()

    def go_to_pose(self, arg_pose):                                     #this function takes the arm to a specific pose

        global flagExecution

        pose_values = self._group.get_current_pose().pose               #for displaying the current pose of the arm

        self._group.set_pose_target(arg_pose)                           #for setting the target pose of the arm
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move  #for taking the arm to the planned pose

        pose_values = self._group.get_current_pose().pose

        list_joint_values = self._group.get_current_joint_values()              #for getting the current joint values of the joints


        if (flag_plan == True):                                 #this flag is used to display whether the planning and the execution was successful
            flagExecution = True
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            flagExecution = False
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
            
            
def main():
    rospy.init_node("percepStack", anonymous=True)
    pub_rgb1 = rospy.Publisher('/center_rgb', String, queue_size = 1)
    pub_depth1 = rospy.Publisher('/center_depth', String, queue_size = 1)
    
    tfBuffer = tf2_ros.Buffer()                                                         #for setting the buffer
    listener = tf2_ros.TransformListener(tfBuffer)

    global pub_rgb
    global pub_depth
    global depth_fin
    pub_rgb = pub_rgb1
    pub_depth = pub_depth1
    
    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    
    sub_image_color_1 = rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck)
    sub_image_depth_1 = rospy.Subscriber("/camera/depth/image_raw2", Image, depth_clbck)
    
    cx , cy = 320.5 , 240.5
    fx , fy = 554.387 , 554.387
        
    hold = 0
    ####################################################################################################
    ur5 = Ur5Moveit()
    while not rospy.is_shutdown():
    
        ur5_pose_1 = geometry_msgs.msg.Pose() 
    
        trans = tfBuffer.lookup_transform('base_link', 'obj'+str(hold), rospy.Time(0))
    
        offset_x_1 = 0.16
        offset_y_1 = 0.4
        offset_z_1 = 0.02

        offset_x_2 = 0.1
        offset_y_2 = 0.25
    
        base_link_Position_X = 0.16                  #this is the x position of the base_link ----> our reference point
        base_link_Position_Y = 0                     #this is the y position of the base_link ----> our reference point
        base_link_Position_Z = 0.53                  #this is the z position of the base_link ----> our reference point
    
        tomato_Position_X_Transform = trans.transform.translation.x           #for storing the x value of transform for the tomato 2
        tomato_Position_Y_Transform = trans.transform.translation.y           #for storing the y value of transform for the tomato 2
        tomato_Position_Z_Transform = trans.transform.translation.z           #for storing the z value of transform for the tomato 2
    
        ur5_pose_1.orientation.x = -0.20426466049594807
        ur5_pose_1.orientation.y = 0.9759094471343439
        ur5_pose_1.orientation.z = -0.07502044797426752
        ur5_pose_1.orientation.w = 0.01576806431223178
    
        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = base_link_Position_X + tomato_Position_X_Transform + offset_x_1
        ur5_pose_1.position.y = base_link_Position_Y + tomato_Position_Y_Transform - offset_y_1
        ur5_pose_1.position.z = base_link_Position_Z + tomato_Position_Z_Transform

        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(2)

        ur5_pose_1 = geometry_msgs.msg.Pose()
        ur5_pose_1.position.x = base_link_Position_X + tomato_Position_X_Transform + offset_x_2
        ur5_pose_1.position.y = base_link_Position_Y + tomato_Position_Y_Transform - offset_y_2
        ur5_pose_1.position.z = base_link_Position_Z + tomato_Position_Z_Transform + offset_z_1
        
        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(2)
        
        ur5.gripper_robot(0)
        
        hold = hold + 1
        
    del ur5
    
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")
