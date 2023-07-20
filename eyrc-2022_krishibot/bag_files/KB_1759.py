#! /usr/bin/env python3


import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit:

    # Constructor
    def __init__(self):
        
        rospy.init_node('fruit_holder', anonymous=True)

        self._planning_group = "arm"
        #self._planning_group.setMaxVelocityScalingFactor(1.0)
        #self._planning_group.setMaxAccelerationScalingFactor(1.0)
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._hand_group = moveit_commander.MoveGroupCommander("gripper")  
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()             
        self._eef_link = self._group.get_end_effector_link() 
        self._group_names = self._robot.get_group_names() 


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
        
    def gripping(self,state):
        if state == 1:
            self._hand_group.set_named_target("close")
            plan2 = self._hand_group.go() 
        elif state == 0:
            self._hand_group.set_named_target("open")
            plan2 = self._hand_group.go()

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    lst_joint_angles_1 = [math.radians(90),
                          math.radians(77),
                          math.radians(-44),
                          math.radians(-4),
                          math.radians(-8),
                          math.radians(1)]

    lst_joint_angles_2 = [math.radians(90),
                          math.radians(63),
                          math.radians(-61),
                          math.radians(15),
                          math.radians(-8),
                          math.radians(1)]


    lst_joint_angles_3 = [math.radians(-15),
                          math.radians(13),
                          math.radians(5),
                          math.radians(18),
                          math.radians(-36),
                          math.radians(1)]


    lst_joint_angles_4 = [math.radians(66),
                          math.radians(55),
                          math.radians(-78),
                          math.radians(-15),
                          math.radians(-18),
                          math.radians(-45)]



    lst_joint_angles_5 = [math.radians(66),
                          math.radians(55),
                          math.radians(-78),
                          math.radians(-15),
                          math.radians(-18),
                          math.radians(-30)]



    lst_joint_angles_6 = [math.radians(-56),
                          math.radians(10),
                          math.radians(4),
                          math.radians(20),
                          math.radians(-26),
                          math.radians(40)]




    while not rospy.is_shutdown():

        ur5.set_joint_angles(lst_joint_angles_1)

        ur5.set_joint_angles(lst_joint_angles_2)
        
        ur5.set_joint_angles(lst_joint_angles_3)
       

        ur5.set_joint_angles(lst_joint_angles_4)
       
        ur5.set_joint_angles(lst_joint_angles_5)
       
        ur5.set_joint_angles(lst_joint_angles_6)
        
       
       

        break

    del ur5


if __name__ == '__main__':
    main()
