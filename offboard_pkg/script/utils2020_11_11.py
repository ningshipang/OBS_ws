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
        self.WIDTH = 640
        self.HEIGHT = 480

        self.circlex = None
        self.circley = None
        self.w, self.h = self.WIDTH, self.HEIGHT
        self.u0 = self.w/2
        self.v0 = self.h/2
        self.x0 = self.u0
        self.y0 = self.v0
        self.f = 554.26  # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2)

        # 判旋转矩阵控制参数
        self.change = 1
        self.k1 = 2.5
        self.k2 = 1.0 #3
        self.k3 = 1.0  # 单独是1
        self.k4 = 0.1  # 0.1是比较缓慢的朝上移动
        self.kf_p = 0.20  # 0.1是比较缓慢的朝上移动  0.12
        self.kf_vod = 10
        self.kf_I = 0.008 #0.008
        self.kfpos_p = 0.1 #0.005
        self.kvod = 0.1 #0.008
        self.integral_v = np.array([0, 0, 0], dtype=np.float64)
        self.integral_pos = np.array([0, 0, 0], dtype=np.float64)
        self.propar_pos = np.array([0, 0, 0], dtype=np.float64)
        self.w1x = 0.1
        self.w1y = 0.1
        self.w1z = 0.1
        self.M = np.array([[self.w1x, 0.0, 0.0], [0.0, self.w1y, 0.0], [0.0, 0.0, self.w1z]])
        self.sat_wb = 10
        self.pitch = 0.01
        self.hover = 0.5673 # real value in real fight
        self.d1 = 0
        self.d2 = 144
        # 判断是否识别到圆的标识符
        self.computer = False
        self.wb2_control = True
        self.wb1_control = True
        self.F_control = "ONLY_F" #ONLY_wb1 ONLY_wb2 ONLY_F HOVER
        self.pos_enable = 0

        # ratate matrix
        # self.sat_x = 2
        # self.sat_y = 2
        # self.sat_z = 2
        # self.aP_x = 1
        # self.aI_x = 1
        # self.aP_y = 1
        # self.aI_y = 1
        # self.aP_z = 1
        # self.aI_z = 1

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

    def Euler_to_RotationMatrix(self, yaw, pitch, roll):
        """
            - function: 欧拉角转旋转矩阵
            - params:
                - pitch: 绕y轴
                - roll: 绕x轴
                - yaw: 绕z轴
            - return:
                -R: 旋转矩阵
        """
        r_11 = math.cos(pitch) * math.cos(yaw)
        r_12 = math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll)
        r_13 = math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)
        r_21 = math.cos(pitch) * math.sin(yaw)
        r_22 = math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll)
        r_23 = math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)
        r_31 = - math.sin(pitch)
        r_32 = math.sin(roll) * math.cos(pitch)
        r_33 = math.cos(roll) * math.cos(pitch)

        q0 = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q1 = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q2 = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        q3 = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)

        R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

        R_rotation = np.array([[r_11, r_12, r_13],
                            [r_21, r_22, r_23],
                            [r_31, r_32, r_33]])
        R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]])
        Rd_rotation = R_rotation.dot(R_ba)
        
        return Rd_rotation

    # def Pos_Rd(self, d_yaw, d_pos, feb_pos, feb_vel):
    #     g = 9.8
    #     e3 = np.array([0, 0, 1])
    #     err_x = d_pos[0] - feb_pos.pose.position.x
    #     err_y = d_pos[1] - feb_pos.pose.position.y
    #     err_z = d_pos[2] - feb_pos.pose.position.z
    #     a_x = self.aP_x * err_x

    def DockingControllerFusion(self, pos_info, pos_i):
        # wx,wy,wz,thrust_min = 0.5
        if pos_info["mav_pos"] == 0:
            return [0, 0, 0, self.hover]

        dt = 0.05 # time interval

        nob = np.array([self.f, pos_i[0] - self.u0, pos_i[1] - self.v0], dtype=np.float64)
        print("u0: {}".format(self.u0))
        print("v0: {}".format(self.v0))
        print("cx: {}".format(pos_i[0] - self.u0))
        print("cy: {}".format(pos_i[1] - self.v0))
        print("nob: {}".format(nob))
        nob /= np.linalg.norm(nob)
        ncb = np.array([1, 0, 0]) 
        no = pos_info["mav_R"].dot(nob)
        print("no: {}".format(no))
        nc = pos_info["mav_R"].dot(ncb)
        # change = 1
        self.change = self.par(no.dot(nc), 0.867, 0.966)  #0.707, 0.867
        if pos_i[1] == 0:
            self.change = 1

        print("change: {}".format(self.change))
        print("angle: {}".format(no.dot(nc)))
        we1 = self.k1 * (np.cross(nc, no))
        wb1 = pos_info["mav_R"].dot(we1)

        rd = self.Euler_to_RotationMatrix(pos_info["mav_original_angle"][0], pos_info["mav_original_angle"][1] + 0.1, pos_info["mav_original_angle"][2])

        wb2 = - vex(self.k2 * ((rd.T).dot(pos_info["mav_R"]) - ((rd.T).dot(pos_info["mav_R"])).T))
        print("wb2: {}".format(self.change * wb2))
        print("wb1: {}".format((1 - self.change) * self.M.dot(wb1)))
        wb = (1 - self.change) * self.M.dot(wb1) + 1.5 * self.change * wb2
        wb = self.sat(wb, self.sat_wb)
        print("wb: {}".format(wb))
        if self.wb1_control == True and self.wb2_control == True:
            wb = wb
            print("wbflag: {}".format(1))
        elif self.wb1_control == False and self.wb2_control == True:
            wb = wb2
        elif self.wb1_control == True and self.wb2_control == False:
            wb = (1 - self.change) * self.M.dot(wb1)
            print("wbflag: {}".format(2))
        else:
            wb = np.array([0, 0, 0])
            print("wbflag: {}".format(3))
        print("wbout: {}".format(wb))
        e3 = np.array([0, 0, 1])
        n3 = pos_info["mav_R"].dot(e3)
        vd1 = matrix(no, no)
        print("n3: {}".format(n3))
        matrix_I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        vI = matrix_I - vd1
        vod = - self.kvod * vI.T.dot(nc)
        print("vod: {}".format(vod))
        
        vd = np.array([0, 0, 0], dtype=np.float64)
        if self.F_control == "ONLY_wb1" or self.F_control == "ONLY_wb2":
            self.change = 1
            # self.pos_enable = (1 - self.change)
        else:
            self.change = self.change
            
        self.pos_enable = self.change

        dlt_v = self.kf_vod * (1 - self.change) * vod - self.change * vd - pos_info["mav_vel"]

        # self.integral_v = self.integral_v + self.kf_I * dlt_v * dt
        # self.SatIntegral(self.integral_v, 0.5, -0.5)

        pos_d = np.array([0, 0, 2.5], dtype=np.float64)
        dlt_pos = pos_d - pos_info["mav_pos"]
        self.propar_pos = self.kfpos_p * dlt_pos
        F_ref = self.hover + self.kf_p * n3.dot(dlt_v) + self.pos_enable * n3.dot(self.propar_pos) 
        F = max(F_ref, self.hover - 0.1)
        print("vod: {}".format(vod))
        print("Fvod: {}".format(self.kf_p * n3.dot(dlt_v)))
        print("n3: {}".format(n3))
        print("dlt_v: {}".format(dlt_v))
        print("Fpos: {}".format(self.pos_enable * n3.dot(self.propar_pos)))
        print("F: {}".format(F))
        print("mav_vel: {}".format(pos_info["mav_vel"]))
        return [wb[0], wb[1], wb[2], F]

    # return a safty vz
    def SaftyZ(self, vz):
        if vz > self.saftyz:
            return self.saftyz
        elif vz < -self.saftyz:
            return -self.saftyz
        else:
            return vz

    def SatIntegral(self, a, up, down):
        for i in range(len(a)):
            if a[i] > up:
                a[i] = up
            elif a[i] < down:
                a[i] = down
        return a

    def Deadband(self, a, deadv):
        for i in range(len(a)):
            if abs(a[i]) < deadv:
                a[i] = 0
            elif a[i] > deadv or a[i] == deadv:
                a[i] = a[i] - deadv
            elif a[i] < - deadv or a[i] == - deadv:
                a[i] = a[i] + deadv
        return a