#!/usr/bin/env python
# -*- coding:utf-8 -*-
#SHIJIANHAN

import numpy as np

class Transformation():
    def __init__(self, A_Shaft_Point, B_Shaft_Point):
        self.A_Shaft_Point = A_Shaft_Point # A 轴点，至少三个
        self.B_Shaft_Point = B_Shaft_Point # B 轴点，至少三个
        self.Lambda = 1 # 默认尺度一样
        self.R = 0
        self.T = 0

    def Trans_Ground_And_Object(self, same = 'Y'):
        '''功能：世界坐标系和机械坐标系的转换
        参数：same 表示两个坐标系的尺度是否一样，默认一样
        输出：旋转矩阵和偏移矩阵
        '''

        # 求 A_Shaft_Point 和 B_Shaft_Point 的中心点
        centroid_A = np.mean(self.A_Shaft_Point,axis=0)
        centroid_B = np.mean(self.B_Shaft_Point,axis=0)

        # 将 A ，B 中心对到一起
        A_move = self.A_Shaft_Point - np.tile(centroid_A,(3,1))
        B_move = self.B_Shaft_Point - np.tile(centroid_B,(3,1))

        # 求 H
        H = np.dot(np.transpose(A_move), B_move)

        # 奇异值分解
        U, _, V = np.linalg.svd(H)

        # 求旋转矩阵
        self.R = np.dot(np.transpose(V), np.transpose(U))
        self.R = np.round((self.R[:,[0,2,1]]), 8) # 控制精度，调整 R 的一个 BUG
        if np.linalg.det(self.R) < 0: # 检查 V 是不是反射矩阵，是的话 V 的第三列乘 -1, 或将R的第三列乘以 -1
            self.R[:,2] = -self.R[:,2]

        # 求偏移矩阵
        self.T = -np.dot(self.R, np.transpose(centroid_A)) + np.transpose(centroid_B)
        self.T = self.T.reshape((3,1))

        # 如果尺度不一致
        # if same == 'N':
        #     A_norm = np.sum(A_move @ A_move, axis=1)
        #     B_norm = np.sum(B_move @ B_move, axis=1)
        #     self.Lambda = A_norm / B_norm
        #     self.Lambda = np.mean(self.Lambda)
        #     self.R = self.R / (np.sqrt(self.Lambda))
        #     self.T = -self.R * np.transpose(centroid_A) + np.transpose(centroid_B) 

        return self.R, self.T

    def check(self, A, B):
        '''功能：检验由 R 、 T 和 A 计算出的 B_1 和 B 间误差 RMSE, RMSE 越接近于0越好
        输入：A, B 两个坐标轴的点
        输出：无
        '''
        N = A.shape[0]
        B_1 = np.dot(self.R, np.transpose(A)) + self.T
        B_1 = np.transpose(B_1)
        err = np.linalg.norm(B-B_1)/N
        print("求出 B_1 为：\n{}\n误差为：{}".format(B_1, err))

    def Trans_Ground_And_Model(self, same = 'Y'):
        pass    

A = np.array([[20,30,20], [20,20,10], [30,40,30]])
B = np.array([[30,20,20], [40,20,10], [20,30,30]])
transform = Transformation(A, B)
R, T = transform.Trans_Ground_And_Object()
print ('旋转矩阵:\n{}\n偏移矩阵:\n{}'.format(R, T))

C = np.array([[10,10,10], [10,50,20], [10,-10,30]])
D = np.array([[50,10,10], [10,10,20], [70,10,30]])
transform.check(C, D)