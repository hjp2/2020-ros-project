#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import commands
import time
import os
import subprocess
import signal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import matplotlib.pyplot as plt


class vacuumbot:  # 메이즈 1차 탈출 클래스
    
    abs_poseX = 0 
    abs_poseY = 0   

    def __init__(self):
        self.enter = True
        self.turn = Twist()
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.state = 3   ##3
        self.state_reverse = False
        self.right_flag = True
        self.startpoint = []
        self.makinglimit = True
        self.done = False

        self.range_ahead = 1
        self.range_left = 1
        self.range_right = 1

        self.aheadlsit = []
        self.rightlist = []
        self.leftlist = []
        self.state0wall = []
        self.stackXY = [] #좌표저장 리스트
        self.stackD = [] #방향 저장 리스트

        self.r = 0.25 #반지름
        self.g = 0.1 #최소 거리 갭
        self.base = self.r + self.g #밑변
        self.height = (self.r * 3) + self.g #높이, 정면 거리
        self.hypo = math.sqrt(math.pow(self.base,2) + math.pow(self.height,2))  #대각선거리
        self.dg = math.degrees(math.atan(self.base/self.height)) #삼각형 각도
        self.rdg = int(math.ceil(self.dg/0.2)) #0.2는 레이저포인트 1 각도        
 

        # 토픽 발행
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)



    # 스캔 콜백 함수
    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[900]
        self.range_left = msg.ranges[1350]  # 1350
        self.range_right = msg.ranges[450]  # 450'''

        #왼쪽 중앙 오른쪽순
        self.aheadlist = [msg.ranges[900-self.rdg], msg.ranges[900], msg.ranges[900+self.rdg]] 
        self.leftlist = [msg.ranges[1350-self.rdg], msg.ranges[1350], msg.ranges[1350+self.rdg]] 
        self.rightlist = [msg.ranges[450-self.rdg], msg.ranges[450], msg.ranges[450+self.rdg]] 

    # 오돔 콜백 함수
    def odom_callback(self, msg):
        self.abs_poseX = msg.pose.pose.position.x
        self.abs_poseY = msg.pose.pose.position.y


    def goal_pose(self):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'odom'
        goal_pose.target_pose.pose.position.x = 0
        goal_pose.target_pose.pose.position.y = 0
        goal_pose.target_pose.pose.position.z = 0
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = 90
        goal_pose.target_pose.pose.orientation.w = 0

        return goal_pose

    ############# 레이저 3개 체크하는 함수
    def checkFront(self): #앞쪽으로 레이저 3개 체크
        if self.aheadlist[0] > self.hypo and self.aheadlist[1] > self.height and self.aheadlist[2] > self.hypo:
            return True
        else:
            return False

    def checkRight(self):
        if self.rightlist[0] > self.hypo and self.rightlist[1] > self.height and self.rightlist[2] > self.hypo:
            return True
        else:
            return False
            
    def checkLeft(self):
        if self.leftlist[0] > self.hypo and self.leftlist[1] > self.height and self.leftlist[2] > self.hypo:
            return True
        else:
            return False
    ##############        

    def save_xy(self):
        if self.makinglimit is False:
            self.stackXY.append([self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)])
        
        if [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)] in self.state0wall:
                self.state0wall.remove([self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)])

    def save_direction(self): #현재 방향 저장하는 함수
        self.stackD.append(self.state)

    def make_cell(self, num):
        tmp = round(num,1)
        tmp2 = str(tmp).split(".")
        result = float(tmp2[0])
        if int(tmp2[1]) >=5:
            if tmp >= 0:
                result = result + 0.5
            if tmp < 0:
                result = result - 0.5

        return result


    def reverse_drive(self): #갈 길이 없으면 뒤로 가는 함수
        self.state_reverse = True
        tmp = (self.stackD.pop()+2)%4 #팝업해서 반대로 돌림

        if abs(self.state - tmp) == 2: #팝업한 방향이 지금 방향이랑 반대일 경우
            self.turn_direction("right", 10)
            self.turn_direction("right", 10)
            
        if self.state - tmp == 3: #머리 동쪽이고 이고 가야될 방향이 북쪽일경우
            self.turn_direction("left", 10)

        if self.state - tmp == -3:
            self.turn_direction("right", 10)

        if self.state - tmp == 1:
            self.turn_direction("right", 10)

        if self.state - tmp == -1:
            self.turn_direction("left", 10)
        
        self.gogo()
        

    #회전하는 함수 ("방향", 시간) 시간 10넣으면 90도 회전 20 넣으면 180도 회전
    def turn_direction(self, direction, time):
        if direction == "right":
            self.turn.angular.z = -math.radians(90)
            self.state = (self.state + 3) % 4 # 회전할때 마다 스테이트 업데이트
        if direction == "left":
            self.turn.angular.z = math.radians(90)
            self.state = (self.state + 1) % 4
        for i in range(time):
            self.cmd_vel_pub.publish(self.turn)
            self.rate.sleep()
        self.turn.angular.z = 0
        self.cmd_vel_pub.publish(self.turn)
        self.rate.sleep()


    def keepgoing(self,dis):
        if self.range_ahead > dis:
            self.twist.linear.x = 0.5
        else:
            self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()
        
    def gogo(self):
        for i in range(10):
            self.twist.linear.x = self.r * 2
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()

        self.save_xy()

        if self.state_reverse is False:
            self.save_direction()


        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()

    #터틀봇 머리가 북쪽일때 검사
    def check0(self):
        if self.checkFront() and [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)+0.5] not in self.stackXY :
            self.state_reverse = False
            self.gogo()
            return

        if self.checkLeft() and [self.make_cell(self.abs_poseX)-0.5, self.make_cell(self.abs_poseY)] not in self.stackXY:
            self.turn_direction("left", 10)
            #self.state = 2
            self.state_reverse = False
            #self.gogo()
            return

        if self.checkRight() and [self.make_cell(self.abs_poseX)+0.5, self.make_cell(self.abs_poseY)] not in self.stackXY:
            self.turn_direction("right", 10)
            #self.state = 4
            self.state_reverse = False
            #self.gogo()
            return

        self.reverse_drive()
        
    #터틀봇 머리가 서쪽일때 검사
    def check1(self):

        if self.checkRight() and [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)+0.5] not in self.stackXY :
            self.turn_direction("right", 10)
            #self.state = 1
            self.state_reverse = False
            #self.gogo()
            return
            

        if self.checkFront() and [self.make_cell(self.abs_poseX)-0.5, self.make_cell(self.abs_poseY)] not in self.stackXY:
            self.state_reverse = False
            self.gogo()
            return

        if self.checkLeft() and [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)-0.5] not in self.stackXY:
            self.turn_direction("left", 10)
            #self.state = 3
            self.state_reverse = False
            #self.gogo()

            return
            

        self.reverse_drive()

    #터틀봇 머리가 남쪽일때 검사
    def check2(self):
        if self.checkRight() and [self.make_cell(self.abs_poseX)-0.5, self.make_cell(self.abs_poseY)] not in self.stackXY:
            self.turn_direction("right", 10)
            #self.state = 2
            self.state_reverse = False
            #self.gogo()
            return
            
        if self.checkFront() and [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)-0.5] not in self.stackXY:
            self.state_reverse = False
            self.gogo()
            return

        if self.checkLeft()  and [self.make_cell(self.abs_poseX)+0.5, self.make_cell(self.abs_poseY)] not in self.stackXY:
            self.turn_direction("left", 10)
            #self.state = 4
            self.state_reverse = False
            #self.gogo()
            return

        self.reverse_drive()

    #터틀봇 머리가 동쪽일때 검사
    def check3(self):

        if self.checkLeft() and [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)+0.5] not in self.stackXY:
            self.turn_direction("left", 10)
            #self.state = 1
            self.state_reverse = False
            #self.gogo()
            return

        if self.checkRight()  and [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)-0.5] not in self.stackXY:
            self.turn_direction("right", 10)
            #self.state = 3
            self.state_reverse = False
            #self.gogo()
            return
            
        if self.checkFront() and [self.make_cell(self.abs_poseX)+0.5, self.make_cell(self.abs_poseY)] not in self.stackXY:
            self.state_reverse = False
            self.gogo()
            return

        self.reverse_drive()
    #처음 토픽 무시되는 버그 고치는 함수
    def nothing(self):
        for i in range(5):
            self.twist.linear.x = 0   
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()


    def makelimit(self):
        
        if self.checkRight():
            self.turn_direction("right",10)

        if self.right_flag:
            if self.checkFront() is False or self.checkRight() is False:
                self.startpoint.append([self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)]) 
                self.right_flag = False
        
       
        if self.checkFront():
            self.gogo()

        else:
            self.turn_direction("left",10)

        if self.state == 0 and self.checkRight() is False:
            self.state0wall.append([self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)])

        if self.right_flag is False and [self.make_cell(self.abs_poseX), self.make_cell(self.abs_poseY)] in self.startpoint:
            self.makinglimit = False
            return
            




        
    def start_bot(self):

        while self.enter:
            self.nothing()
            #mand = "rosrun gmapping slam_gmapping"
            #p = subprocess.Popen(mand, shell=True)
     
            self.enter = False

        while self.makinglimit:
            self.makelimit()

        while self.makinglimit is False and len(self.state0wall) > 0:
            if self.state == 0:
                self.check0()
            if self.state == 1:
                self.check1()
            if self.state == 2:
                self.check2()
            if self.state == 3:
                self.check3()

        if  self.makinglimit is False and len(self.state0wall) == 0 and self.done is False:
            te = "cd ~/robotvacuum/ && rosrun map_server map_saver -f map && convert map.pgm map.png && eog map.png"
            os.system(te)
            time.sleep(1)

            x, y = zip(*self.stackXY)
            plt.plot(x, y, linewidth=6)
            plt.axis('off')

            #plt.scatter(x, y)
            plt.savefig('/home/hjp/robotvacuum/route.png')
            os.system('eog /home/hjp/robotvacuum/route.png')
            #plt.show()
            print ("cleaned area:", len(self.stackXY) * self.r * self.r, "m^2")
            print ("goal start")
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
            client.wait_for_server()
            time.sleep(1)
            client.send_goal(self.goal_pose())
            client.wait_for_result()

            self.done = True
            #os.killpg(os.getpgid(p.pid), signal.SIGINT)


class start_bot:

    def __init__(self):
        self.bot = vacuumbot()


    def begin(self):
        while not rospy.is_shutdown():
            self.bot.start_bot()

            if self.bot.done:
                break



rospy.init_node('vacuumbot')

start_clean = start_bot()

if __name__ == "__main__":
    start_clean.begin()