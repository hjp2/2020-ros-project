#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math



class Mazebot:  # 메이즈 1차 탈출 클래스
    xyxy = []
    abs_poseX = 0  # 주어진 맵의 좌표
    abs_poseY = 0
  

    def __init__(self):
        self.enter = True
        self.turn = Twist()
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.state = 3 
        self.right_flag = True
        self.left_flag = True
        self.wall_flag = False
        self.endflag = False
        self.gotogoal = True
        self.gotoHome = False

        self.range_ahead = 1
        self.range_left = 1
        self.range_right = 1

        self.aheadlsit = []
        self.rightlist = []
        self.leftlist = []
        self.walllist = []

        self.r = 0.1 #반지름
        self.g = 0.4 #최소 거리 갭
        self.base = self.r + self.g#밑변
        self.height = (self.r + 0.2) + self.g #높이, 정면 거리
        self.hypo = math.sqrt(math.pow(self.base,2) + math.pow(self.height,2))  #대각선거리
        self.dg = math.degrees(math.atan(self.base/self.height)) #삼각형 각도
        self.rdg = int(math.ceil(self.dg/0.2)) #0.2는 레이저포인트 1 각도        

        self.poseX = 0   # 오돔 좌표
        self.poseY = 0

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)


    def scan_callback(self, msg):

        self.range_ahead = msg.ranges[900]
        self.range_left = msg.ranges[1350]
        self.range_right = msg.ranges[450]

        #왼쪽 중앙 오른쪽순
        self.aheadlist = [msg.ranges[900-self.rdg], msg.ranges[900], msg.ranges[900+self.rdg]] 
        self.leftlist = [msg.ranges[1350-self.rdg], msg.ranges[1350], msg.ranges[1350+self.rdg]] 
        self.rightlist = [msg.ranges[450-self.rdg], msg.ranges[450], msg.ranges[450+self.rdg]] 

    # 오돔 좌표와 지도의 좌표를 계산, 오돔 콜백 함수
    def odom_callback(self, msg):
        self.poseX = msg.pose.pose.position.x
        self.poseY = msg.pose.pose.position.y
        self.abs_poseX = self.poseX - 7.92
        self.abs_poseY = self.poseY + 5.30


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
        
        
    #회전하는 함수 ("방향", 시간) 시간 10넣으면 90도 회전 20 넣으면 180도 회전
    def turn_direction(self, direction, time):
        if direction == "right":
            self.turn.angular.z = -math.radians(90)
            self.state = (self.state + 3) % 4 
        if direction == "left":
            self.turn.angular.z = math.radians(90)
            self.state = (self.state + 1) % 4
        for i in range(time):
            self.cmd_vel_pub.publish(self.turn)
            self.rate.sleep()
        self.wall_flag = True
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
            self.twist.linear.x = 0.5
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()

        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.rate.sleep()



    #처음 토픽 무시되는 버그? 고치는 함수
    def nothing(self):
        for i in range(5):
            self.twist.linear.x = 0   
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
   

    def leftlazer0(self):
        myx = round(self.abs_poseX,1)
        myy = round(self.abs_poseY,1)
        wx = 0
        wy = 0
        
        flag = False

        for i in self.xyxy:
            if myx -0.8 < i[0] < myx and i[1] == myy:
                flag = True
                wx = i[0]
                wy = i[1]
                break

        if flag:
            if self.wall_flag:
                self.walllist.append([myx, myy-0.1, 1])         

    def leftlazer1(self):
        myx = round(self.abs_poseX,1)
        myy = round(self.abs_poseY,1)
        flag = False
        wx = 0
        wy = 0

        for i in self.xyxy:
            if myx == i[0] and myy-0.8 < i[1] < myy:
                flag = True
                wx = i[0]
                wy = i[1]
                break

        if flag:
            if self.wall_flag:
                self.walllist.append([myx+0.1, myy, 2])


    def leftlazer2(self):
        myx = round(self.abs_poseX,1)
        myy = round(self.abs_poseY,1)
        flag = False
        wx = 0
        wy = 0

        for i in self.xyxy:
            if myx < i[0] < myx+0.8 and i[1] == myy:
                flag = True
                wx = i[0]
                wy = i[1]
                break

        if flag:
            if self.wall_flag:
                self.walllist.append([myx, myy+0.1, 3])


    def leftlazer3(self):
        myx = round(self.abs_poseX,1)
        myy = round(self.abs_poseY,1)
        flag = False
        wx = 0
        wy = 0

        for i in self.xyxy:
            if myx == i[0] and myy < i[1] < myy+0.8:
                flag = True
                wx = i[0]
                wy = i[1]
                break

        if flag:
            if self.wall_flag:
                self.walllist.append([myx-0.1, myy, 0])


    def check_sigh(self):
        tmp = -1
        for i in range(len(self.walllist)):
            if round(max(self.walllist[i][0],round(self.abs_poseX,1)) - min(self.walllist[i][0],round(self.abs_poseX,1)),1) < 0.2 and round(max(self.walllist[i][1],round(self.abs_poseY,1)) - min(self.walllist[i][1],round(self.abs_poseY,1)),1) < 0.2:

                tmp = self.walllist[i][2]
                break

        if self.left_flag and tmp >=0:
            if self.state - tmp == 3: 
                self.turn_direction("left", 10)

            if self.state - tmp == -3:
                self.turn_direction("right", 10)

            if self.state - tmp == 1:
                self.turn_direction("right", 10)

            if self.state - tmp == -1:
                self.turn_direction("left", 10)
            


            self.left_flag = False


        
    def maze_run(self):
        #미로찾기
        if self.enter:
            self.nothing()
            self.turn_direction("right",10)
            self.gogo()
            self.enter = False

        if self.gotogoal:
            if self.checkRight() and  self.right_flag:
                self.turn_direction("right",10)
                self.xyxy.append([round(self.abs_poseX,1),round(self.abs_poseY,1)])
                self.right_flag = False

            if self.checkRight() is False:
                self.right_flag = True

            if self.range_ahead < self.base+0.1:
                self.turn_direction("left",10)

            self.keepgoing(0.225)

            if self.state == 0:
                self.leftlazer0()
            if self.state == 1:
                self.leftlazer1()
            if self.state == 2:
                self.leftlazer2()
            if self.state == 3:
                self.leftlazer3()

            if self.abs_poseY < -5.0 :
                self.gotogoal = False
                self.gotoHome = True
                self.turn_direction("right", 10)
                self.turn_direction("right", 10)

        if self.gotoHome:
            if self.checkLeft() and  self.left_flag:
                self.turn_direction("left",10)
                self.left_flag = False

            if self.checkLeft() is False:
                self.left_flag = True

            self.check_sigh()
                     
            if self.range_ahead < self.base+0.1:
                self.turn_direction("right",10)

            if self.abs_poseY > 4:
                self.endflag = True
            
            self.keepgoing(0.225)




class start_bot:

    def __init__(self):
        self.bot = Mazebot()


    def begin(self):
        while not rospy.is_shutdown():

            self.bot.maze_run()

            if self.bot.endflag:
                break




rospy.init_node('mazebot')

start_maze = start_bot()

if __name__ == "__main__":
    start_maze.begin()