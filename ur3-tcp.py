#!/usr/bin/env python
# -*- coding:utf-8 -*-
#Author: nulige
# coding=utf8
# coding:utf8

from socket import *
import struct
import array
import time as T
ip_port=('192.168.0.1',30003)
back_log=5
buffer_size=1060

tcp_client=socket(AF_INET,SOCK_STREAM)
tcp_client.connect(ip_port)


def check_pose(pose):
    """检查以确保姿势是有效的。
       检查姿势是否是一个6成员元组或浮点数列表，姿势无效时引发异常。
    """
    if not isinstance(pose, (tuple, list)):
        raise TypeError("Expected tuple or list for pose")
    if not all([isinstance(x, float) for x in pose]):
        raise TypeError("Expected floats in pose")
    if not len(pose) == 6:
        raise TypeError("Expected 6 members in pose")

def deg_2_rad(x):
    """度转弧度
    """
    return 3.14 * x / 180

def rad_2_deg(x):
    """弧度转度
    """
    x = list(x)
    for i in range(len(x)):
        x[i] = (x[i] / 3.14) * 180
    return tuple(x)

def write_data():
    '''写入的是各个关节的角度
    '''
    input_data = input("输入移动位置,不要加空格：(x,y,z,rx,ry,rz)\n")
    input_data = str(input_data)[1:-1].split(',')
    for i in range(len(input_data)):
        input_data[i] = float(input_data[i])
    return input_data

def movej(__socket): 
    '''移动
    '''
    cartesian = False
    input_data = write_data()
    check_pose(input_data)
    clean_list_tuple = map(deg_2_rad, input_data)
    a_joint = 1.4 # 加速度
    v_joint = 0.75 # 速度
    t = 0 # 移动时间
    radius =0 # 弧度
    message = 'movej({}{},a={},v={},t={},r={})'.format('q' if cartesian
                                                            else '',
                                                            clean_list_tuple,
                                                            a_joint,
                                                            v_joint,
                                                            t,
                                                            radius)+'\n'
    __socket.send(message)
    T.sleep(2.5) # 每次命令发送后需要等待移动

def read_data():
    '''读取数据：返回运行时间
                返回工具在基坐标系下的坐标
                返回各个关节角度
    '''
    # format 格式：
    # ！转为大端
    # I 解析数据长度
    # I 132d 解析全部数据
    formatLength = struct.Struct('! I')
    format = struct.Struct("! I 132d")

    recv_bytes = tcp_client.recv(buffer_size) # 接收数据
    print(formatLength.unpack(recv_bytes[0:4])[0])
    
    clean_data = array.array('.2d', [0] * 132)
    clean_data = format.unpack(recv_bytes) # 解析成数据
    time = clean_data[1] # 取出机械臂运行时间
    tool_vector_actual = clean_data[56:61] # 取出工具在基坐标系下的坐标
    q_actual = rad_2_deg(clean_data[32:38]) # 取出各个关节角度

    print(u'开启时间：%s,工具在base下的坐标: %s,各个关节角度: %s \n'%(
        time/60, 
        tool_vector_actual,
        q_actual
        )) # 加u不会乱码

if __name__ == "__main__":

    read_data()
    
    movej(tcp_client)

    read_data()



tcp_client.close()


