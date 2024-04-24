#!/usr/bin/python3
import time
from tf import transformations
import rospy
import tf
import copy

class TaskPoint:
    """Property and method about a task point.
    Attributes:
        pose: the pose of robot in this wappoint.
        name: waypoint name. 
    """
    def __init__(self,record = None):
        if not record:
            record = {
                "order" : 0,
                "robot_pose" : {
                    "pos_x" : 0.0,
                    "pos_y" : 0.0,
                    "pos_z" : 0.0,
                    "ori_x" : 0.0,
                    "ori_y" : 0.0,
                    "ori_z" : 0.0,
                    "ori_w" : 1.0
                }
            }
        self.record = copy.deepcopy(record)
        # self.tf_listener = tf.TransformListener()
        self.update_waypoint_name()
        
    def setPreTaskPoint(self,src_point):
        self.pre_task_point = src_point
    
    def getPreTaskPoint(self):
        return self.pre_task_point

    
    def update_waypoint_name(self):
        self.name = "waypoint_" + str(self.record["order"])

    def setRobotPose(self,robot_pose):
        self.record["robot_pose"]["pos_x"] = robot_pose[0]
        self.record["robot_pose"]["pos_y"] = robot_pose[1]
        self.record["robot_pose"]["pos_z"] = robot_pose[2]

        self.record["robot_pose"]["ori_x"] = robot_pose[3]
        self.record["robot_pose"]["ori_y"] = robot_pose[4]
        self.record["robot_pose"]["ori_z"] = robot_pose[5]
        self.record["robot_pose"]["ori_w"] = robot_pose[6]


    def getPosX(self):
        robot_pose = self.record["robot_pose"]
        return robot_pose["pos_x"]

    def getPosY(self):
        robot_pose = self.record["robot_pose"]
        return robot_pose["pos_y"]

    def getPosZ(self):
        robot_pose = self.record["robot_pose"]
        return robot_pose["pos_z"]

    def getOrder(self):
        return self.record["order"]

    def getYaw(self):
        robot_pose = self.record["robot_pose"]
        pose = []
        pose.append(robot_pose["pos_x"])
        pose.append(robot_pose["pos_y"])
        pose.append(robot_pose["pos_z"])
        pose.append(robot_pose["ori_x"])
        pose.append(robot_pose["ori_y"])
        pose.append(robot_pose["ori_z"])
        pose.append(robot_pose["ori_w"])
        return transformations.euler_from_quaternion(pose[3:])[2]

    def getPoseList(self):
        robot_pose = self.record["robot_pose"]
        pose = []
        pose.append(robot_pose["pos_x"])
        pose.append(robot_pose["pos_y"])
        pose.append(robot_pose["pos_z"])
        pose.append(robot_pose["ori_x"])
        pose.append(robot_pose["ori_y"])
        pose.append(robot_pose["ori_z"])
        pose.append(robot_pose["ori_w"])
        return pose

    def calDistance(self,other):
        '''
            For Simplifacation, using 2D distance as the pose distance.
        '''
        return (
                (self.getPosX()-other.getPosX())**2 + (self.getPosY()-other.getPosY())**2
               )**0.5


    # def listen_tf(self):
    #     try:
    #         (pos,ori) = self.tf_listener.lookupTransform("/map","/base_link",rospy.Duration(0.0))
    #         print("pos: ",pos)
    #         print("ori: ",ori)
    #         msg_list =[
    #             pos[0],pos[1],pos[2],
    #             ori[0],ori[1],ori[2],ori[3]
    #         ]
    #         return msg_list
    #     except tf.Exception as e:
    #         print("listen to tf failed")
    #         print(e)
    #         print(e.message)
    #         return None
         
    def runTask(self):
        """
            When robot successfully tranfer to a new TaskPoint, Correspoing operation task will be triggered.
            eg. shoot an image. 
        """
        rospy.loginfo("Run task in {}.".format(self.name))
        rospy.sleep(0.05)

        
