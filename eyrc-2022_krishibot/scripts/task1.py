#!/usr/bin/env python3
import rospy                                            #for importing rospy
from geometry_msgs.msg import Twist                     #for getting the geometry messages of type Twist
from sensor_msgs.msg import LaserScan                   #to get the laser values
from nav_msgs.msg import Odometry                       #for getting the odometry

def control_loop():          #the control function

    rospy.init_node('task1')             #initializing node 

    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)   #setting up the publisher
    ### sends to topic /cmd_vel

    rospy.Subscriber('/ebot/laser/scan',LaserScan, laser_callback)      #setting up the subscriber
    ### gets from topic /ebot/laser/scan

    rospy.Subscriber('/odom', Odometry, odom_callback)  #setting up the subscriber

    rate =  rospy.Rate(10)      #defining the rate for the exectuion


    velocity_msg = Twist()

    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    pub.publish(velocity_msg)


    #getting all the global variables

    while not rospy.is_shutdown(): ### as long as ctrl C is not done in terminal

        rate.sleep()


if __name__=='__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
