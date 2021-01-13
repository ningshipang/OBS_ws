#!/usr/bin/env python
#coding=utf-8

import numpy as np
from avoidance import Avoidance
import math
import tf

# 定义点的函数
class Point():
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def getx(self):
        return self.x

    def gety(self):
        return self.y

# 定义直线函数
class Getlen():
    def __init__(self, p1, p2):
        self.x = p1.getx() - p2.getx()
        self.y = p1.gety() - p2.gety()
        # 用math.sqrt（）求平方根
        self.len = math.sqrt((self.x ** 2) + (self.y ** 2))

    # 定义得到直线长度的函数
    def getlen(self):
        return self.len


def vex(matrix_A):
    return np.array([matrix_A[2, 1], matrix_A[0, 2], matrix_A[1, 0]])


def matrix(B, C):
    return np.array([[B[0] * C[0], B[0] * C[1], B[0] * C[2]],
                     [B[1] * C[0], B[1] * C[1], B[1] * C[2]],
                     [B[2] * C[0], B[2] * C[1], B[2] * C[2]]])


class Utils(object):
    def __init__(self, params):
        self.WIDTH = 640 #simulation 720
        self.HEIGHT = 480 #simulation 405

        self.circlex = None
        self.circley = None
        self.w, self.h = self.WIDTH, self.HEIGHT
        self.u0 = self.w/2
        self.v0 = self.h/2
        self.x0 = self.u0
        self.y0 = self.v0
        #realsense: fx:632.9640658678117  fy:638.2668942402212
        self.f = 632 #346.6  # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2),不同仿真环境以及真机实验中需要依据实际情况进行修改
        #camrea frame to mavros_body frame
        self.R_cb = np.array([[1,0,0],\
                             [0,0,1],\
                             [0,-1,0]])
        self.R_origin = np.array([[0,0,1],\
                             [1,0,0],\
                             [0,1,0]])
        self.n_cc = np.array([0,0,1])

        self.vd = np.array([0, 0, 0], dtype=np.float64)
        self.ko = 2.5 #the angular rate to avoid obstacle

        self.kp_vd = 2.0 #the p_control about desire velicoty

        self.kvod = 2.5 #the p_control about desire velicoty by matrix initial value:1


    def distance(self, circle):
        p1 = Point(circle[0], circle[1])
        p2 = Point(self.w / 2, self.h / 2)
        l = Getlen(p1, p2)
        # print(self.circley)
        return l.getlen()

    def distance_y(self, circle):
        len_y = circle[1] - self.h / 2
        return len_y

    def par(self, d, d1, d2):
        if d < d1 or d == d1:
            return 1.0
        elif (d1 < d and d < d2) or d == d2:
            A = -2 / math.pow((d1 - d2), 3)
            B = 3 * (d1 + d2) / math.pow((d1 - d2), 3)
            C = -6 * d1 * d2 / math.pow((d1 - d2), 3)
            D = (math.pow(d2, 2) * (3 * d1 - d2)) / math.pow((d1 - d2), 3)
            return A * math.pow(d, 3) + B * math.pow(d, 2) + C * math.pow(d, 1) + D
        else:
            return 0.0

    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a

    def DockingControllerFusion(self, pos_info, pos_i):
        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)
        if pos_i[1] != 0:
            print("n_eo:{}".format(n_eo))
            print("theta: {}".format(n_eo.dot(n_ec)))
            im = [(pos_i[0]-self.u0), -(pos_i[1]-self.v0)]
            print("pos_i: {}".format(im))
        # print("n_ec:{}".format(n_ec))
        # n_original = self.R_origin.dot(n_co)
        # n_pri = pos_info["mav_R"].dot(n_original)
        # print("n_pri:{}".format(n_pri))

        self.change = self.par(n_eo.dot(n_ec), 0.867, 0.966)  #0.707, 0.867 the parameter about smooth
        if pos_i[1] == 0:
            self.change = 1
        
        #the calculation of the angular to avoid obstacle
        # w_eo = self.ko * (np.cross(n_ec, n_eo))
        # w_bo = pos_info["mav_R"].T.dot(w_eo) #pos_info["mav_R"].T.dot(we1)

        #calculate vod
        vd1 = matrix(n_eo, n_eo)
        matrix_I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        vI = matrix_I - vd1
        vod = (1 - self.change) * self.kvod * vI.T.dot(n_ec)
        print("vod:{}".format(vod))
        # if pos_i[1] != 0:
        #     print("vod:{}".format(vod))
        #desire velicoty
        # self.vd = self.kp_vd * (vod - pos_info["mav_vel"])
        v_rd_b = np.array([1, 0, 0])
        v_rd_e = pos_info["mav_R"].dot(v_rd_b)
        print("vr:{}".format(v_rd_e))

        v = vod + self.change * v_rd_e
        print("v:{}".format(v))
        return [v[0], v[1], v[2], 0]

    #期望位置，反馈位置，位置比例系数，速读限幅
    def pos_control(self, target_pos, feb_pos, kp, sat_vel):
        err_pos = target_pos - feb_pos
        cmd_pos_vel = self.sat(kp * err_pos, sat_vel)
        # cmd_pos_vel[2] = 
        return [cmd_pos_vel[0], cmd_pos_vel[1], cmd_pos_vel[2]]
        
    #期望位置，反馈位置，反馈角度，偏航角控制比例系数，角速度限幅
    def yaw_control(self, target_yaw, feb_yaw, kp_yaw, sat_yaw):
        #机头指向目标点的位置
        # desire_yaw = math.atan2(target_pos[1] - feb_pos[1], target_pos[0] - feb_pos[0])
        dlt_yaw = self.minAngleDiff(target_yaw, feb_yaw)
        cmd_yaw = self.Saturation(kp_yaw * dlt_yaw, sat_yaw, -sat_yaw)
        return cmd_yaw

    def minAngleDiff(self, a, b):
        diff = a - b
        if diff < 0:
            diff += 2*np.pi
        if diff < np.pi:
            return diff
        else:
            return diff - 2*np.pi

    # return a safty vz
    def SaftyZ(self, vz, satf):
        if vz > satf:
            return satf
        elif vz < -satf:
            return -satf
        else:
            return vz

    def SatIntegral(self, a, up, down):
        for i in range(len(a)):
            if a[i] > up:
                a[i] = up
            elif a[i] < down:
                a[i] = down
        return a

    def Saturation(self, a, up, down):
        if a > up:
            a = up
        elif a < down:
            a = down
        return a

    def PID_Control(self):
        p =1

    def Deadband(self, a, deadv):
        for i in range(len(a)):
            if abs(a[i]) < deadv:
                a[i] = 0
            elif a[i] > deadv or a[i] == deadv:
                a[i] = a[i] - deadv
            elif a[i] < - deadv or a[i] == - deadv:
                a[i] = a[i] + deadv
        return a