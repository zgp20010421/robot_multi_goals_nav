#!/usr/bin/python3
#coding: utf-8

import os
import json
import rospy
from TaskPoint import TaskPoint
from TaskTransfer import TaskTransfer
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose,PoseArray
import numpy as np
import tf
import rospkg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Task:
    def __init__(self):
        self.taskPoints = []
        self.currentIndex = 0
        self.robot_transfer = None
        self.src_ind = None
        self.des_ind = None
        self.package_path = None
        self.mark_pub = None
        self.task_pose_pub = None
        self.ntask = 0
        # rospy.Subscriber("/iniialpose", PoseWithCovarianceStamped, self.initPose_callback)

    def init_rospack(self):
        #get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        #list all packages,,equivalent to rospack list
        rospack.list()
        try:
            self.package_path = rospack.get_path('sophon_robot')
        except:
            print("Could not find package sophon_robot")
            exit(1)
        print(self.package_path)
            
    def init(self):
        self.mark_pub = rospy.Publisher('/robot_path',MarkerArray,queue_size = 100)
        self.task_pose_pub = rospy.Publisher('/robot_path_pose',PoseArray,queue_size = 100)
        task_init = TaskInit()                                                  #任务初始化类
        self.init_rospack()
        self.robot_transfer = TaskTransfer()
        self.loadTaskPoints()

        #plot all task points
        self.plot_marker()

        best_ind,initial_point = task_init.getBestTaskInd(self.taskPoints)
        #First to transfer to best index point  'self.taskPoints[best_index]'
        self.robot_transfer.task_transfer(initial_point,self.taskPoints[best_ind])

        self.taskPoints[best_ind].runTask()
        self.ntask = self.taskPoints.__len__()
        self.src_ind = best_ind
        self.des_ind = (best_ind + 1) % self.ntask

    def plot_marker(self):
        markerArray = MarkerArray()
        for point in self.taskPoints:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.pose.position.x = point.getPosX()
            marker.pose.position.y = point.getPosY()
            marker.pose.position.z = point.getPosZ()
            marker.text = str(point.getOrder())
            markerArray.markers.append(marker)    

        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
            
        # self.mark_pub(markerArray)

    def loadTaskPoints(self):
        '''
                Construct TaskPoint list from json files in data folder.
        '''
        folder = '{}/data'.format(self.package_path)
        task_json = None
        if os.path.exists(folder):
            task_json=os.listdir(folder)
        
        if not task_json:
            raise Exception("No valid task point to tranverse!!")

        task_list = []
        for i,file_name in enumerate(task_json):
            with open(folder + '/' + file_name,'r') as json_fp:
                waypoint_record = json.load(json_fp)
                task_list.append(waypoint_record)
        task_list = sorted(task_list,key = lambda s : s['order'])
        for waypoint_record in task_list:
            self.taskPoints.append(
                TaskPoint(waypoint_record)
            )

    def run(self):
        '''
            Main loop
        '''
        while not rospy.is_shutdown():
            self.robot_transfer.task_transfer(self.taskPoints[self.src_ind],self.taskPoints[self.des_ind])
            self.taskPoints[self.des_ind].runTask()
            self.src_ind = self.des_ind
            self.des_ind = (self.des_ind + 1) % self.ntask


class TaskInit():
    def __init__(self, initpose_topic="/amcl_pose"):
       self.initialPose = None
       self.pose_topic = initpose_topic
       rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.amcl_pose_callback)
    #    self.tf_listener = tf.TransformListener()

    def amcl_pose_callback(self, data):
        msg_list = [
                data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,
                data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                data.pose.pose.orientation.w
            ]
        self.initialPose = msg_list
    
    # def listen_tf(self):
    #     try:
    #         (pos,ori) = self.tf_listener.lookupTransform("/map","base_link",rospy.Duration(0.0))
    #         print("pos:",pos)
    #         print("ori:",ori)
    #         msg_list = [
    #             pos[0],pos[1],pos[2],
    #             ori[0],ori[1],ori[2],ori[3]
    #         ]
    #         self.initialPose = msg_list
    #         return True
    #     except tf.Exception as e:
    #         print("listen to tf failed")
    #         print(e)
    #         print(e.message)
    #         return False

    def getBestTaskInd(self,task_points):
        # self.refreshInitialPose()
        fake_task = TaskPoint()
        fake_task.setRobotPose(self.initialPose)
        dist_list = [fake_task.calDistance(task_point) for task_point in task_points]
        return np.argmin(np.array(dist_list)), fake_task

if __name__=="__main__":
    rospy.init_node("auto_2d_nav_task")
    task = Task()
    task.init()
    task.run()