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
        # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2),不同仿真环境以及真机实验中需要依据实际情况进行修改
        self.WIDTH = 720
        self.HEIGHT = 405

        self.circlex = None
        self.circley = None
        self.w, self.h = self.WIDTH, self.HEIGHT
        self.u0 = self.w/2
        self.v0 = self.h/2
        self.x0 = self.u0
        self.y0 = self.v0
        self.f = 554.26  # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2),不同仿真环境以及真机实验中需要依据实际情况进行修改
        #camrea frame to mavros_body frame
        self.R_cb = np.array([[1,0,0],\
                             [0,0,1],\
                             [0,-1,0]])
        self.R_origin = np.array([[0,0,1],\
                             [1,0,0],\
                             [0,1,0]])
        self.n_cc = np.array([0,0,1])

        self.vd = np.array([0, 0, 0], dtype=np.float64)
        self.ko = 4.0 #the angular rate to avoid obstacle:2.5
        self.kd = 1.0 #small angule parameter:1.0
        self.w1x = 0.3 #0.1
        self.w1y = 0.3 #0.1
        self.w1z = 0.5 #0.2
        self.M = np.array([[self.w1x, 0.0, 0.0], [0.0, self.w1y, 0.0], [0.0, 0.0, self.w1z]])

        self.sat_wb = 5

        self.kp_vd = 2.0 #the p_control about desire velicoty

        self.kvod = 1 #the p_control about desire velicoty by matrix

        self.integral_v = np.array([0, 0, 0], dtype=np.float64)
        self.integral_pos = np.array([0, 0, 0], dtype=np.float64)
        self.propar_pos = np.array([0, 0, 0], dtype=np.float64)
        self.Fpos = 0
        self.kfpos_p = 0.3 #0.2
        self.hover = 0.60905 # real value in real fight
        self.kf_p = 1  # 0.1是比较缓慢的朝上移动  0.12
        


    def distance(self, circle):
        p1 = Point(circle[0], circle[1])
        p2 = Point(self.w / 2, self.h / 2)
        l = Getlen(p1, p2)
        # print(self.circley)
        return l.getlen()

    def distance_y(self, circle):
        len_y = circle[1] - self.h / 2
        return len_y

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
        w_eo = self.ko * (np.cross(n_eo, n_ec))
        w_bo = (1/(1 - n_eo.dot(n_ec))) * pos_info["mav_R"].T.dot(w_eo) #pos_info["mav_R"].T.dot(we1)
        
        #the calculation of the angular to keep desired attitude
        '''
        the relationship between rd and vd
        [0,0.1,0]       [0,4,0]
        [0,0.4,0]       [0,9.3,0]
        [0,0.15,0]       [0,6,0]
        [0,1,0]       [0,20,0]

        [0.3,0,0]       [0,0,0]

        [0,0,0.1]       [4,0,0]
        [0,0,0.4]       [9.3,0,0]
        '''
        rd = self.Euler_to_RotationMatrix(pos_info["mav_original_angle"][0], pos_info["mav_original_angle"][1] + 0.1, pos_info["mav_original_angle"][2])   #change the attitude
        vd = np.array([0, 4, 0], dtype=np.float64)
        
        w_bd = - vex(self.kd * ((rd.T).dot(pos_info["mav_R"]) - ((rd.T).dot(pos_info["mav_R"])).T))

        wb = (1 - self.change) * self.M.dot(w_bo) + self.change * w_bd
        wb = self.sat(wb, self.sat_wb)
        print("wb: {}".format(wb))
        print("w_bd: {}".format(w_bd))
        print("w_bo: {}".format(w_bo))

        #calculate vod
        e3 = np.array([0, 0, 1])
        # n3 = pos_info["mav_R"].T.dot(e3) #pos_info["mav_R"].T.dot(e3)
        n3 = pos_info["mav_R"].dot(e3) #pos_info["mav_R"].T.dot(e3)
        vd1 = matrix(n_eo, n_eo)
        matrix_I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        vI = matrix_I - vd1
        vod = (1 - self.change) * self.kvod * vI.T.dot(n_ec)
        
        dlt_v = vod + self.change * vd - pos_info["mav_vel"]
        pos_d = np.array([pos_info["Initial_pos"][0], pos_info["Initial_pos"][1], pos_info["Initial_pos"][2]], dtype=np.float64)
        dlt_pos = pos_d - pos_info["mav_pos"]
        print("posd:{}".format(pos_d))
        print("mav_pos:{}".format(pos_info["mav_pos"]))
        print("dlt_pos:{}".format(dlt_pos))
        self.propar_pos = self.kfpos_p * dlt_pos
        self.Fpos = self.SaftyZ(self.change * e3.dot(self.propar_pos), 0.1) #saturation Fpos   0.3
        Fv = self.kf_p * n3.dot(dlt_v)
        Fv = self.SaftyZ(Fv, 0.4)  #0.4
        F_ref = self.hover + Fv + self.Fpos # self.pos_enable * n3.dot(self.propar_pos) 
        # F_ref = self.hover + self.Fpos # self.pos_enable * n3.dot(self.propar_pos)
        F = max(F_ref, self.hover - 0.1)
        F = self.SaftyZ(F, 0.63) #saturation F  0.63
        # F = self.SaftyZ(F, 0.8) #saturation F
        print("F:{}".format(F))
        print("Fpos:{}".format(self.Fpos))
        print("Fv:{}".format(Fv))

        # return [vod[0], vod[1], vod[2], 0]
        # return [w_bo[0], w_bo[1], w_bo[2], F]
        return [wb[0], wb[1], wb[2], F]
        # return [0, 0, 0, F]

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