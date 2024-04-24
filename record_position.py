#!/usr/bin/env python3
#coding: utf-8

import sys
import rospy
from PyQt5.QtWidgets import QWidget,QApplication,QLabel,QHBoxLayout,QVBoxLayout,QPushButton,QLineEdit,QTextEdit
from PyQt5.QtGui import QColor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import subprocess
import json
import os
import tf
import rospkg

class  LocationRecorder(QWidget):
      def __init__(self):
            super(LocationRecorder,self).__init__()
            self.init_ui()
            self.current_robot_pose = None
            self.robot_record_pose = None
            self.package_path = None
            self.pose_dict = None
            self.markerArray = MarkerArray()
            self.markerArray_number = MarkerArray()
            self.init_rospack()
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
            self.mark_pub = rospy.Publisher('/robot_path',MarkerArray,queue_size = 100)
            self.show()
      
      def init_rospack(self):
            rospack = rospkg.RosPack()
            rospack.list()
            try:
                  self.package_path = rospack.get_path('sophon_robot')
            except:
                  print("Could not find package sophon_robot")
                  exit(1)
            print(self.package_path)
            

      def init_ui(self):
            self.layout = QVBoxLayout()
            self.order_layout = QHBoxLayout()
            self.order_layout.addWidget(QLabel("位置编号："))
            self.order_edit = QLineEdit("")
            self.order_layout.addWidget(self.order_edit)

            self.text_content = QTextEdit()
            self.text_content.setEnabled(False)

            self.record_layout = QHBoxLayout()
            self.receive_button = QPushButton("获取位点")
            self.record_button = QPushButton("记录位点")
            self.nav_button = QPushButton("开始导航")
            self.close_button = QPushButton("取消导航")
            self.record_layout.addWidget(self.receive_button)
            self.record_layout.addWidget(self.record_button)
            self.record_layout.addWidget(self.nav_button)
            self.record_layout.addWidget(self.close_button)

            self.layout.addLayout(self.order_layout)
            self.layout.addWidget(self.text_content)
            self.layout.addLayout(self.record_layout)
            self.setLayout(self.layout)

            self.record_button.clicked.connect(self.record)
            self.receive_button.clicked.connect(self.receive)
            self.nav_button.clicked.connect(self.multi_goals_nav)
            self.close_button.clicked.connect(self.close_nav)

      def record(self):
            order = self.order_edit.text()
            try:
                  order = int(order)
            except:
                  return
            
            new_record = {
                  "order" : order,
                  "robot_pose" : self.robot_record_pose
            }

            os.system("mkdir -p {}/data".format(self.package_path))
            with open("{}/data/{}.json".format(self.package_path,order),'w+') as out:
                  json.dump(new_record,out,indent=4)
                  
            marker = Marker()      #创建marker对象
            marker.header.frame_id = 'map' #以哪一个TF坐标为原点
            marker.type = marker.ARROW #一直面向屏幕的字符格式
            marker.action = marker.ADD #添加marker
            marker.scale.x = 0.5 #marker大小
            marker.scale.y = 0.05 #marker大小
            marker.scale.z = 0.05 #marker大小，对于字符只有z起作用
            marker.color.a = 1 #字符透明度
            marker.color.r = 16 #字符颜色R(红色)通道
            marker.color.g = 1 #字符颜色G(绿色)通道
            marker.color.b = 0 #字符颜色B(蓝色)通道
            marker.pose.position.x = new_record["robot_pose"]["pos_x"] #字符位置
            marker.pose.position.y = new_record["robot_pose"]["pos_y"] #字符位置
            marker.pose.orientation.z = new_record["robot_pose"]["ori_z"] #字符位置
            marker.pose.orientation.w = new_record["robot_pose"]["ori_w"] #字符位置
            self.markerArray.markers.append(marker) #添加元素进数组

            #markers的id不能一样，否则rviz只会识别最后一个元素
            id = 0
            for m in self.markerArray.markers:    #遍历marker分别给id赋值
                  m.id = id
                  id += 1

            marker_number = Marker()      #创建marker对象
            marker_number.header.frame_id = 'map' #以哪一个TF坐标为原点
            marker_number.type = marker_number.TEXT_VIEW_FACING #一直面向屏幕的字符格式
            marker_number.action = marker_number.ADD #添加marker
            marker_number.scale.x = 0.5 #marker大小
            marker_number.scale.y = 0.5 #marker大小
            marker_number.scale.z = 0.5 #marker大小，对于字符只有z起作用
            marker_number.color.a = 1 #字符透明度
            marker_number.color.r = 0 #字符颜色R(红色)通道
            marker_number.color.g = 0.5 #字符颜色G(绿色)通道
            marker_number.color.b = 16 #字符颜色B(蓝色)通道
            marker_number.pose.position.x = new_record["robot_pose"]["pos_x"] #字符位置
            marker_number.pose.position.y = new_record["robot_pose"]["pos_y"] #字符位置
            marker_number.pose.position.z = 0.1 #字符位置
            marker_number.pose.orientation.z = 0 #字符位置
            marker_number.pose.orientation.w = 1 #字符位置
            marker_number.text = str(new_record["order"]) #字符内容
            self.markerArray_number.markers.append(marker_number) #添加元素进数组
            
            for m in self.markerArray_number.markers:    #遍历marker分别给id赋值
                  m.id = id
                  id += 1

            self.mark_pub.publish(self.markerArray)
            self.mark_pub.publish(self.markerArray_number)
            self.text_content.setTextColor(QColor('green'))
            self.text_content.setText('record Success!!!')
      
      def goal_callback(self, data):
            self.pose_dict = {
                    "pos_x" : data.pose.position.x,
                    "pos_y" : data.pose.position.y,
                    "pos_z" : data.pose.position.z,
                    "ori_x" : data.pose.orientation.x,
                    "ori_y" : data.pose.orientation.y,
                    "ori_z" : data.pose.orientation.z,
                    "ori_w" : data.pose.orientation.w
            }
            self.current_robot_pose = self.pose_dict
            # print('self.pose_dict: {}'.format(self.current_robot_pose))

      def receive(self):
            self.robot_record_pose = self.current_robot_pose
            display_msg = "Robot:\n" + json.dumps(self.robot_record_pose, indent=4) + "\n"
            self.text_content.setTextColor(QColor('black'))
            self.text_content.setText(display_msg)
      
      def multi_goals_nav(self):
            # 执行命令启动multi_goals_nav.launch文件
            subprocess.Popen(["roslaunch", "sophon_robot", "multi_goals_nav.launch"])

      
      def close_nav(self):
            # 执行命令停止multi_goals_nav.launch文件
            subprocess.call(["rosnode", "kill", "/multi_goals_nav"])

if  __name__ == "__main__":
    app = QApplication(sys.argv)
    rospy.init_node("record_location")
    lr = LocationRecorder()
    sys.exit(app.exec_())