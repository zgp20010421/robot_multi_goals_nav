#!/usr/bin/python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from TaskPoint import TaskPoint
from geometry_msgs.msg import PoseStamped

class TaskTransfer:

    def __init__(self):
        self.moveToClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.moveToClient.wait_for_server()
        rospy.loginfo("Action 'MoveTo' is up !")

    def task_transfer(self,src_point,des_point):
        '''
            Transfer robot to next TaskPoint.
        '''
        des_point.setPreTaskPoint(src_point)
        goal_msg = MoveBaseGoal()

        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"
        poseList = des_point.getPoseList()
        goal_pose.pose.position.x = poseList[0]
        goal_pose.pose.position.y = poseList[1]
        goal_pose.pose.position.z = poseList[2]
        goal_pose.pose.orientation.x =poseList[3]
        goal_pose.pose.orientation.y = poseList[4]
        goal_pose.pose.orientation.z = poseList[5]
        goal_pose.pose.orientation.w = poseList[6]

        goal_msg.target_pose = goal_pose

        print("-----------------------------------------------")
        print("goal_msg.target_pose.pose.position:",goal_msg.target_pose.pose.position)
        print("goal_msg.target_pose.pose.orientation:",goal_msg.target_pose.pose.orientation)
        print("--------------------------------------------------")

        not_done = True
        while not rospy.is_shutdown() and not_done:
            self.moveToClient.send_goal(goal_msg)
            rospy.logwarn("Transfer from [%s] to [%s]"%(src_point.name,des_point.name))
            done = self.moveToClient.wait_for_result(timeout = rospy.Duration(300.0))
            # Make sure the action succeed
            not_done = (not done) or (self.moveToClient.gh.get_goal_status() != actionlib.GoalStatus.SUCCEEDED)
            print("Goal status:" ,self.moveToClient.gh.get_goal_status == actionlib.GoalStatus.SUCCEEDED)
            if not_done:
                print("Goal not done,Resent goal.")
