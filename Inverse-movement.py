#!/usr/bin/env python
# -*- coding:utf-8 -*-
#SHIJIANHAN
#UR3的正逆运动
import cv2
import numpy as np
import scipy.linalg as la


class inverse_move():
    def __init__(self):
        # 初始参数
        self.ALPHA = np.array([0, np.pi/2, 0, 0, np.pi/2, -np.pi/2]) # alpha_i-1, rad, alpha有0没有6
        self.D = np.array([0.1519, 0, 0, 0.11235, 0.08535, 0.0819]) # d_i, mm
        self.A = np.array([0, 0, -0.24365, -0.21325, 0, 0]) # a_i-1, mm
        self.theta = np.zeros((8,6+1)) # 用于放最后求的参数, rad
        self.Output = {} # 用于输出

    def deg2rad(self, inx):
        '''度转弧度
        '''
        outy = [i * np.pi / 180 for i in inx]
        return outy

    def positive(self, theta_input):
        '''正运动，由角度到坐标
        参数：theta_input是机械臂六个角度
        输出：tool在base下的坐标
        '''
        T = np.zeros((6,4,4)) # 定义六个4x4的T矩阵
        for i in range(1,T.shape[0]+1):
            T[i-1, 0, 0] = np.cos(theta_input[i])
            T[i-1, 0, 1] = -np.sin(theta_input[i])  
            T[i-1, 0, 2] = 0
            T[i-1, 0, 3] = self.A[i-1] 
            T[i-1, 1, 0] = np.sin(theta_input[i]) * np.cos(self.ALPHA[i-1])  
            T[i-1, 1, 1] = np.cos(theta_input[i]) * np.cos(self.ALPHA[i-1])    
            T[i-1, 1, 2] = -np.sin(self.ALPHA[i-1])    
            T[i-1, 1, 3] = -self.D[i-1] * np.sin(self.ALPHA[i-1])         
            T[i-1, 2, 0] = np.sin(theta_input[i]) * np.sin(self.ALPHA[i-1])
            T[i-1, 2, 1] = np.cos(theta_input[i]) * np.sin(self.ALPHA[i-1])
            T[i-1, 2, 2] = np.cos(self.ALPHA[i-1])
            T[i-1, 2, 3] = self.D[i-1] * np.cos(self.ALPHA[i-1]) 
            T[i-1, 3, 0] = 0
            T[i-1, 3, 1] = 0
            T[i-1, 3, 2] = 0
            T[i-1, 3, 3] = 1
        T06 = T[0]@T[1]@T[2]@T[3]@T[4]@T[5]
        a, _ = cv2.Rodrigues(T06[0:3,0:3])
        # print('T06矩阵:\n %s'%T06)
        print("正运动计算结果：tool在base下坐标： (%.3f mm, %.3f mm, %.3f mm)"%(T06[0, 3]*1000, T06[1, 3]*1000, T06[2, 3]*1000))
        # print('姿态为：(%.3f, %.3f, %.3f)'%(a[0], a[1], a[2]))

    def inverse(self, position):
        '''UR3逆运动
        参数：position是[x, y, z, Rx, Ry, Rz]
        输出：8组角度解
        '''
        assert len(position) == 6

        # 输入单位为mm， 转为m    
        x = position[0] / 1000
        y = position[1] / 1000
        z = position[2] / 1000

        # 由旋转向量求出旋转矩阵
        rotation_matrix, _ = cv2.Rodrigues((position[3], position[4], position[5]))

        # 求theta_1, rad ===============================================================
        A = rotation_matrix[0,2] * self.D[5] - x
        B = rotation_matrix[1,2] * self.D[5] - y
        C = self.D[3]
        # theta_1_1分给前4组
        self.theta[0, 1] = np.arctan2(B, A) - np.arctan2(C, np.sqrt(A**2+B**2-C**2))
        self.theta[1, 1] = np.arctan2(B, A) - np.arctan2(C, np.sqrt(A**2+B**2-C**2))
        self.theta[2, 1] = np.arctan2(B, A) - np.arctan2(C, np.sqrt(A**2+B**2-C**2))
        self.theta[3, 1] = np.arctan2(B, A) - np.arctan2(C, np.sqrt(A**2+B**2-C**2))
        # theta_1_2分给后4组
        self.theta[4, 1] = np.arctan2(B, A) - np.arctan2(C, -np.sqrt(A**2+B**2-C**2))
        self.theta[5, 1] = np.arctan2(B, A) - np.arctan2(C, -np.sqrt(A**2+B**2-C**2))
        self.theta[6, 1] = np.arctan2(B, A) - np.arctan2(C, -np.sqrt(A**2+B**2-C**2))
        self.theta[7, 1] = np.arctan2(B, A) - np.arctan2(C, -np.sqrt(A**2+B**2-C**2))

        # 求theta_5_1, rad ==============================================================
        A = np.sin(self.theta[0, 1]) * rotation_matrix[0, 2] - np.cos(self.theta[0, 1]) * rotation_matrix[1, 2]
        # theta_5_1_1分给0，1组
        self.theta[0, 5] = np.arctan2(np.sqrt(1 - A * A), A)
        self.theta[1, 5] = self.theta[0, 5]
        # theta_5_1_2分给2，3组
        self.theta[2, 5] = np.arctan2(-np.sqrt(1 - A * A), A)
        self.theta[3, 5] = self.theta[2, 5]
        # 求theta_5_2, rad
        A = np.sin(self.theta[4, 1]) * rotation_matrix[0, 2] - np.cos(self.theta[4, 1]) * rotation_matrix[1, 2]
        # theta_5_2_1分给4，5组
        self.theta[4, 5] = np.arctan2(np.sqrt(1 - A * A), A)
        self.theta[5, 5] = self.theta[4, 5]
        # theta_5_2_2分给6，7组
        self.theta[6, 5] = np.arctan2(-np.sqrt(1 - A * A), A)
        self.theta[7, 5] = self.theta[6, 5]

        # 求theta_6, rad ===============================================================
        for i in range(8):
            A = (-np.sin(self.theta[i, 1]) * rotation_matrix[0, 1] + np.cos(self.theta[i, 1]) * rotation_matrix[1, 1]) /  np.sin(self.theta[i, 5])###
            B = (np.sin(self.theta[i, 1]) * rotation_matrix[0, 0] - np.cos(self.theta[i, 1]) * rotation_matrix[1, 0]) /  np.sin(self.theta[i, 5])
            self.theta[i, 6] = np.arctan2(A, B)
        
        # 求theta_2, theta_3, theta_4, rad ==============================================
        for i in range(0, 8, 2):

            # 求theta234, rad
            theta234 = np.zeros(8)
            A = - rotation_matrix[2, 2] / np.sin(self.theta[i, 5])
            B = - (np.cos(self.theta[i, 1]) * rotation_matrix[0, 2] + np.sin(self.theta[i, 1] * rotation_matrix[1, 2])) / np.sin(self.theta[i, 5])###
            theta234[i] = np.arctan2(A, B) - np.pi
            theta234[i+1] = theta234[i]

            # 求theta_2, rad
            A = - np.cos(theta234[i]) * np.sin(self.theta[i, 5]) * self.D[5] + np.sin(theta234[i]) * self.D[4]
            B = - np.sin(theta234[i]) * np.sin(self.theta[i, 5]) * self.D[5] - np.cos(theta234[i]) * self.D[4]
            C = np.cos(self.theta[i, 1]) * x + np.sin(self.theta[i ,1]) * y
            D = z - self.D[0]
            M = C - A
            N = D - B
            E = -2 * N * self.A[2]
            F = 2 * M * self.A[2]
            G = M**2 + N**2 + self.A[2]**2 - self.A[3]**2
            self.theta[i, 2] = np.arctan2(F, E) - np.arctan2(G, np.sqrt(E**2+F**2-G**2))
            self.theta[i+1, 2] = np.arctan2(F, E) - np.arctan2(G, - np.sqrt(E**2+F**2-G**2))

            # 求theta23，rad
            theta23 = np.zeros(8)
            A = (N - np.sin(self.theta[i, 2]) * self.A[2]) / self.A[3]
            B = (M - np.cos(self.theta[i, 2]) * self.A[2]) / self.A[3]
            C = (N - np.sin(self.theta[i+1, 2]) * self.A[2]) / self.A[3]
            D = (M - np.cos(self.theta[i+1, 2]) * self.A[2]) / self.A[3]
            theta23[i] = np.arctan2(A, B)
            theta23[i+1] = np.arctan2(C, D)

            # 求theta_3, rad
            self.theta[i, 3] = theta23[i] - self.theta[i, 2]
            self.theta[i+1, 3] = theta23[i+1] - self.theta[i+1, 2]

            # 求theta_4, rad
            self.theta[i, 4] = theta234[i] - theta23[i]
            self.theta[i+1, 4] = theta234[i+1] - theta23[i+1]

        # 输出结果并检验结果
        for k in range(8):
            outmeg = "第{}组解：\ntheta_1:{:.3f}   theta_2:{:.2f}   theta_3:{:.2f}   theta_4:{:.2f}   theta_5:{:.2f}   theta_6:{:.2f}".format(
                k,
                self.theta[k, 1] * 180 / np.pi,
                self.theta[k, 2] * 180 / np.pi,
                self.theta[k, 3] * 180 / np.pi,
                self.theta[k, 4] * 180 / np.pi,
                self.theta[k, 5] * 180 / np.pi,
                self.theta[k, 6] * 180 / np.pi
                )
            print(outmeg)
            self.positive(self.theta[i]) 
        

        # 对结果进行取舍
        kk = 0
        for k in range(8):
            JUDGE = 1
            if self.theta[k, 1] * 180 / np.pi < -90 or self.theta[k, 1] * 180 / np.pi > 90 or self.theta[k, 1] == None:
                JUDGE = 0
            if self.theta[k, 2] * 180 / np.pi < -90 or self.theta[k, 2] * 180 / np.pi > 90 or self.theta[k, 2] == None :
                JUDGE = 0
            if self.theta[k, 3] * 180 / np.pi < -90 or self.theta[k, 3] * 180 / np.pi > 90 or self.theta[k, 3] == None :
                JUDGE = 0
            if self.theta[k, 4] * 180 / np.pi < -90 or self.theta[k, 4] * 180 / np.pi > 90 or self.theta[k, 4] == None :
                JUDGE = 0
            if self.theta[k, 5] * 180 / np.pi < -90 or self.theta[k, 5] * 180 / np.pi > 90 or self.theta[k, 5] == None :
                JUDGE = 0
            if self.theta[k, 6] * 180 / np.pi < -90 or self.theta[k, 6] * 180 / np.pi > 90 or self.theta[k, 6] == None :
                JUDGE = 0
            if JUDGE :
                self.Output[kk] = self.theta[k]
                kk = kk+1

        smeg = [str(j) + ": " + ",".join(str(i*180/np.pi) for i in v) for (j, v) in self.Output.items()]
        print("取舍后可用组为: %s"%smeg[0])








        

        



'''
def positive2(theta_input):
    正运动，由角度到坐标
    参数：theta_input是机械臂六个角度
    输出：tool在base下的坐标
    
    T = np.zeros((6,4,4)) # 定义六个4x4的T矩阵
    for i in range(1,T.shape[0]+1):
        T[i-1, 0, 0] = np.cos(theta_input[i])
        T[i-1, 0, 1] = -np.sin(theta_input[i]) * np.cos(ALPHA[i-1])   
        T[i-1, 0, 2] = np.sin(theta_input[i]) * np.sin(ALPHA[i-1])
        T[i-1, 0, 3] = A[i-1] * np.cos(theta_input[i]) 
        T[i-1, 1, 0] = np.sin(theta_input[i]) 
        T[i-1, 1, 1] = np.cos(theta_input[i]) * np.cos(ALPHA[i-1])    
        T[i-1, 1, 2] = -np.cos(theta_input[i]) * np.sin(ALPHA[i-1])    
        T[i-1, 1, 3] = A[i-1] * np.sin(theta_input[i])         
        T[i-1, 2, 0] = 0
        T[i-1, 2, 1] = np.sin(ALPHA[i-1])
        T[i-1, 2, 2] = np.cos(ALPHA[i-1])
        T[i-1, 2, 3] = D[i-1]
        T[i-1, 3, 0] = 0
        T[i-1, 3, 1] = 0
        T[i-1, 3, 2] = 0
        T[i-1, 3, 3] = 1
    T06 = T[0]@T[1]@T[2]@T[3]@T[4]@T[5]
    print(T06)
    print("正运动计算结果：tool在base下坐标为(%f, %f, %f)"%(T06[0, 3]*1000, T06[1, 3]*1000, T06[2, 3]*1000))
'''




inverse = inverse_move()

rad = inverse.deg2rad([0, -35.23, -77.88, -19.85, -110.03, 46.70, 97.95])
inverse.positive(rad)
inverse.inverse([-40.67, -179.28, 648.91, 1.1961, 1.7012, -1.0646])
