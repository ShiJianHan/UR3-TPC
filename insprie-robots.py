#!/usr/bin/env python
# -*- coding:utf-8 -*-
#Author: nulige
# coding=utf8
# coding:utf8

import time
import serial
import serial.tools.list_ports

class InspireHandR:
    def __init__(self):
        #串口设置
        plist = list(serial.tools.list_ports.comports())
        if len(plist) <= 0:
            print("串口没找到")
        else:
            plist_0 = list(plist[0])
            serialName = plist_0[0]
            self.ser = serial.Serial(serialName, 115200)
            # print("check which port was really used >", self.ser)
        self.ser.isOpen()
        self.hand_id = 1
        power1 = 100
        power2 = 200
        power3 = 300
        power4 = 400
        power5 = 500
        self.setpower(power1,power2,power3,power4,power5)
        
        speed1 = 600
        speed2 = 600
        speed3 = 600
        speed4 = 600
        speed5 = 600
        speed6 = 600
        self.setspeed(speed1,speed2,speed3,speed4,speed5,speed6) 

        self.f1_init_pos = 2000    #小指初始位置
        self.f2_init_pos = 2000    #无名指初始位置
        self.f3_init_pos = 2000    #中指初始位置
        self.f4_init_pos = 2000   #食指指初始位置
        self.f5_init_pos = 1000    #拇指初始位置
        self.f6_init_pos = 1000   #拇指转向掌心初始位置

        # 手部张开，测试用
        # self.f1_init_pos = 0    #小指初始位置
        # self.f2_init_pos = 0    #无名指初始位置
        # self.f3_init_pos = 0    #中指初始位置
        # self.f4_init_pos = 0    #食指指初始位置
        # self.f5_init_pos = 0    #拇指初始位置
        # self.f6_init_pos = 0    #拇指转向掌心初始位置

        self.reset()
 
    #把数据分成高字节和低字节
    def data2bytes(self,data):
        rdata = [0xff]*2
        if data == -1:
            rdata[0] = 0xff
            rdata[1] = 0xff
        else:
            rdata[0] = data&0xff
            rdata[1] = (data>>8)&(0xff)
        return rdata

    #把十六进制或十进制的数转成bytes
    def num2str(self,num):
        str = hex(num)
        str = str[2:4]
        if(len(str) == 1):
            str = '0'+ str
        # str = bytes.fromhex(str)
        str = bytes.fromhex(str)     
        #print(str)
        return str
   
    #求校验和,是除应答帧头外其余数据的累加和的低字节。
    def checknum(self,data,leng):
        result = 0
        for i in range(2,leng):
            result += data[i]
            
        result = result&0xff
        #print(result)
        return result

    def setpower(self,power1,power2,power3,power4,power5):
        '''设置抓力阈值 
        功能：主控单元设置灵巧手 5 根手指的指尖抓力阈值。
        指令帧长度：16Bytes
        指令号：0x52（CMD_MC_SET_DRVALL_YBP_THRESHOLD）
        数据内容：5 根手指的指尖抓力阈值（从小指依次到大拇指的顺序，以克为单位），每
                个阈值为 2Bytes（小端模式低字节先发送），共 10Bytes，阈值的有效值为 0~1000，若为
                0xFFFF，则表示不需要设置该手指的阈值，因此可单独设置某根手指的指尖阈值。
        '''
        if power1 <0 or power1 >1000:
            print('数据超出正确范围：0-1000')
            return
        if power2 <0 or power2 >1000:
            print('数据超出正确范围：0-1000')
            return
        if power3 <0 or power3 >1000:
            print('数据超出正确范围：0-1000')
            return
        if power4 <0 or power4 >1000:
            print('数据超出正确范围：0-1000')
            return
        if power5 <0 or power5 >1000:
            print('数据超出正确范围：0-1000')
            return
        
        datanum = 0x0B
        b = [0]*(datanum + 5)
        #包头
        b[0] = 0xEB
        b[1] = 0x90

        #hand_id号
        b[2] = self.hand_id

        #数据体长度
        b[3] = datanum
        
        #写操作
        b[4] = 0x52

        #数据
        b[5] = self.data2bytes(power1)[0]
        b[6] = self.data2bytes(power1)[1]
        
        b[7] = self.data2bytes(power2)[0]
        b[8] = self.data2bytes(power2)[1]
        
        b[9] = self.data2bytes(power3)[0]
        b[10] = self.data2bytes(power3)[1]
        
        b[11] = self.data2bytes(power4)[0]
        b[12] = self.data2bytes(power4)[1]
        
        b[13] = self.data2bytes(power5)[0]
        b[14] = self.data2bytes(power5)[1]
        
        #校验和
        b[15] = self.checknum(b,datanum+5)
        
        #向串口发送数据
        putdata = b''
        
        for i in range(1,datanum+6):
            putdata = putdata + self.num2str(b[i-1])
        self.ser.write(putdata)
        time.sleep(0.5)
        # print('设置的抓力阈值：%s'%putdata.hex(" "))
        read_num = self.ser.inWaiting()
        getdata= self.ser.read(read_num)
        if getdata.hex(" ").split(" ")[5] == "01":
            print("抓力阈值指令成功接收")
        else:
            print("抓力阈值指令接受失败")
        # print('设置抓力阈值返回的数据：%s'%getdata.hex(" "))

    def setspeed(self,speed1,speed2,speed3,speed4,speed5,speed6):
        '''设置速度 
        功能：主控单元设置灵巧手中 6 个直线驱动器的运动速度。
        指令帧长度：18Bytes
        指令号：0x51（CMD_MC_SET_DRVALL_SPEED）
        数据内容：6 个驱动器的速度，每个速度为 2Bytes（小端模式低字节先发送），共
                12Bytes，速度的有效值为 0~1000，若为 0xFFFF，则表示不需要设置该驱动器的速度，因
                此可单独设置某个驱动器的速度值。
        '''
        if speed1 <0 or speed1 >1000:
            print('数据超出正确范围：0-1000')
            return
        if speed2 <0 or speed2 >1000:
            print('数据超出正确范围：0-1000')
            return
        if speed3 <0 or speed3 >1000:
            print('数据超出正确范围：0-1000')
            return
        if speed4 <0 or speed4 >1000:
            print('数据超出正确范围：0-1000')
            return
        if speed5 <0 or speed5 >1000:
            print('数据超出正确范围：0-1000')
            return
        if speed6 <0 or speed6 >1000:
            print('数据超出正确范围：0-1000')
            return
        
        datanum = 0x0D
        b = [0]*(datanum + 5)
        #包头
        b[0] = 0xEB
        b[1] = 0x90

        #hand_id号
        b[2] = self.hand_id

        #数据个数
        b[3] = datanum
        
        #指令号
        b[4] = 0x51
        
        #数据
        b[5] = self.data2bytes(speed1)[0]
        b[6] = self.data2bytes(speed1)[1]
        
        b[7] = self.data2bytes(speed2)[0]
        b[8] = self.data2bytes(speed2)[1]
        
        b[9] = self.data2bytes(speed3)[0]
        b[10] = self.data2bytes(speed3)[1]
        
        b[11] = self.data2bytes(speed4)[0]
        b[12] = self.data2bytes(speed4)[1]
        
        b[13] = self.data2bytes(speed5)[0]
        b[14] = self.data2bytes(speed5)[1]
        
        b[15] = self.data2bytes(speed6)[0]
        b[16] = self.data2bytes(speed6)[1]
        
        #校验和
        b[17] = self.checknum(b,datanum+5)
        
        #向串口发送数据
        putdata = b''
        
        for i in range(1,datanum+6):
            putdata = putdata + self.num2str(b[i-1])
        self.ser.write(putdata)
        # print('设置速度发送的数据：%s'%putdata.hex())
        time.sleep(0.5)
        read_num = self.ser.inWaiting()
        getdata= self.ser.read(read_num)
        if getdata.hex(" ").split(" ")[5] == "01":
            print("速度指令成功接收")
        else:
            print("速度指令接受失败")
        # print('设置速度返回的数据：%s'%getdata.hex())

    def setangle(self,angle1,angle2,angle3,angle4,angle5,angle6):
        '''设置目标归一化角度
        功能：主控单元设置灵巧手中 6 个自由度的关节角度，已经进行了归一化处理，数据
            范围从 0 到 1000,0 为握拳状态下各自由度能达到的极限角度，1000 为五指伸开状态下各自
            由度能达到的极限角度
        指令帧长度：18Bytes
        指令号：0x54（CMD_MC_SET_DRVALL_SEEKANGLE_GYH）
        数据内容：6 个自由度的角度数据，每个角度数据为 2Bytes（小端模式低字节先发送），
                共 12Bytes，角度的数据范围是从 0 到 1000，若为 0xFFFF，则表示不需要设置该驱动器的
                目标角度，因此可单独设置某个驱动器的目标角度。
        '''
        if angle1 <0 or angle1 >1000:
            print('数据超出正确范围：0-1000')
            return
        if angle2 <0 or angle2 >1000:
            print('数据超出正确范围：0-1000')
            return
        if angle3 <0 or angle3 >1000:
            print('数据超出正确范围：0-1000')
            return
        if angle4 <0 or angle4 >1000:
            print('数据超出正确范围：0-1000')
            return
        if angle5 <0 or angle5 >1000:
            print('数据超出正确范围：0-1000')
            return
        if angle6 <0 or angle6 >1000:
            print('数据超出正确范围：0-1000')
            return
        
        datanum = 0x0D
        b = [0]*(datanum + 5)
        #包头
        b[0] = 0xEB
        b[1] = 0x90

        #hand_id号
        b[2] = self.hand_id

        #数据体长度
        b[3] = datanum
        
        #指令号
        b[4] = 0x54
        
        #数据
        b[5] = self.data2bytes(angle1)[0]
        b[6] = self.data2bytes(angle1)[1]
        
        b[7] = self.data2bytes(angle2)[0]
        b[8] = self.data2bytes(angle2)[1]
        
        b[9] = self.data2bytes(angle3)[0]
        b[10] = self.data2bytes(angle3)[1]
        
        b[11] = self.data2bytes(angle4)[0]
        b[12] = self.data2bytes(angle4)[1]
        
        b[13] = self.data2bytes(angle5)[0]
        b[14] = self.data2bytes(angle5)[1]
        
        b[15] = self.data2bytes(angle6)[0]
        b[16] = self.data2bytes(angle6)[1]
        
        #校验和
        b[17] = self.checknum(b,datanum+5)
        
        #向串口发送数据
        putdata = b''
        
        for i in range(1,datanum+6):
            putdata = putdata + self.num2str(b[i-1])
        self.ser.write(putdata)
        print('设置目标归一化角度发送的数据：%s'%putdata.hex())
        read_num = self.ser.inWaiting()
        getdata= self.ser.read(read_num)
        print('设置目标归一化角度返回的数据：%s'%getdata.hex())

    def setpos(self,pos1,pos2,pos3,pos4,pos5,pos6):
        '''功能：主控单元设置灵巧手中 6 个直线驱动器的目标位置，使灵巧手完成相应的手势
                动作。灵巧手中的 6 个直线伺服驱动器的 ID 号为 1-6，其中小拇指的 ID 为 1、无名指的 ID
                为 2、中指的 ID 为 3、食指的 ID 为 4、大拇指弯曲指关节 ID 为 5、大拇指旋转指关节 ID
                为 6。
        指令帧长度：18Bytes
        指令号：0x50（CMD_MC_SET_DRVALL_SEEKPOS）
        数据内容：6 个驱动器的目标位置，每个位置为 2Bytes（小端模式低字节先发送），共12Bytes，目标位置的有效值为 0~2000，若为 0xFFFF，则表示不需要设置该驱动器的目标
                位置，因此可单独设置某个驱动器的目标位置
        '''
        global hand_id
        if pos1 <0 or pos1 >2000:
            print('数据超出正确范围：0-2000')
            return
        if pos2 <0 or pos2 >2000:
            print('数据超出正确范围：0-2000')
            return
        if pos3 <0 or pos3 >2000:
            print('数据超出正确范围：0-2000')
            return
        if pos4 <0 or pos4 >2000:
            print('数据超出正确范围：0-2000')
            return
        if pos5 <0 or pos5 >2000:
            print('数据超出正确范围：0-2000')
            return
        if pos6 <0 or pos6 >2000:
            print('数据超出正确范围：0-2000')
            return
        
        datanum = 0x0D
        b = [0]*(datanum + 5)
        #包头
        b[0] = 0xEB
        b[1] = 0x90

        #hand_id号
        b[2] = self.hand_id

        #数据体长度
        b[3] = datanum
        
        #写操作
        b[4] = 0x50
        
        #数据
        b[5] = self.data2bytes(pos1)[0]
        b[6] = self.data2bytes(pos1)[1]
        
        b[7] = self.data2bytes(pos2)[0]
        b[8] = self.data2bytes(pos2)[1]
        
        b[9] = self.data2bytes(pos3)[0]
        b[10] = self.data2bytes(pos3)[1]
        
        b[11] = self.data2bytes(pos4)[0]
        b[12] = self.data2bytes(pos4)[1]
        
        b[13] = self.data2bytes(pos5)[0]
        b[14] = self.data2bytes(pos5)[1]
        
        b[15] = self.data2bytes(pos6)[0]
        b[16] = self.data2bytes(pos6)[1]
        
        #校验和
        b[17] = self.checknum(b,datanum+5)
        
        #向串口发送数据
        putdata = b''
        
        for i in range(1,datanum+6):
            putdata = putdata + self.num2str(b[i-1])
        self.ser.write(putdata)
        # print('设置目标位置发送的数据：%s'%putdata.hex())
        time.sleep(0.5)
        read_num = self.ser.inWaiting()
        getdata= self.ser.read(read_num)
        if getdata.hex(" ").split(" ")[5] == "01":
            print("目标位置指令成功接收")
        else:
            print("目标位置指令接受失败")
        # print('设置目标位置返回的数据：%s'%getdata.hex())
        return

    def reset(self):
        '''复位
        功能：回到最初位置
        '''
        pos1 = self.f1_init_pos #小拇指伸直0，弯曲2000
        pos2 = self.f2_init_pos #无名指伸直0，弯曲2000
        pos3 = self.f3_init_pos #中指伸直0，弯曲2000
        pos4 = self.f4_init_pos #食指伸直0，弯曲2000
        pos5 = self.f5_init_pos #大拇指伸直0，弯曲2000
        pos6 = self.f6_init_pos #大拇指转向掌心 2000
        self.setpos(pos1,pos2,pos3,pos4,pos5,pos6) 
        return

        #读取驱动器实际的位置值
   
    def get_target_position(self):        
        '''读取目标位置
        功能：主控单元读取灵巧手当前 6 个驱动器的目标位置（0--2000）。
        指令帧长度：6Bytes
        指令号：0xD0（CMD_MC_READ_DRVALL_SEEKPOS）
        数据内容：无
        '''
        datanum = 0x01
        b = [0]*6
        #包头
        b[0] = 0xEB
        b[1] = 0x90

        #手的id号
        b[2] = self.hand_id

        #数据体长度
        b[3] = datanum

        #指令号
        b[4] = 0xD0

        #校验和
        b[5] = self.checknum(b, 5+datanum)
        
        #向串口发送数据
        putdata = b''
        
        for i in range(1,datanum+6):
            putdata = putdata + self.num2str(b[i-1])
        self.ser.write(putdata)

        # print('读取目标位置发送的数据：%s'%putdata.hex(" "))
        time.sleep(0.5)
        read_num = self.ser.inWaiting()
        getdata = self.ser.read(read_num)
        # print('读取目标位置返回的数据：%s'%getdata.hex(" "))

        Data = getdata.hex(" ").split(" ")
        setpos = [0.0]*6
        for i in range(1,12):
            if i%2 == 0:
                continue
            else:
                s1 = Data[5+i-1]
                s2 = Data[5+i]
                s = s2 + s1
                setpos[int((i-1)/2)] = int(s, 16)
        print('目标位置为：(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)'%(setpos[0], setpos[1], setpos[2], setpos[3], setpos[4], setpos[5]))

    def get_current_position(self):        
        '''读取当前位置
        功能：主控单元读取灵巧手当前 6 个驱动器的当前位置（0--2000）。
        指令帧长度：6Bytes
        指令号：0xD0（CMD_MC_READ_DRVALL_SEEKPOS）
        数据内容：无
        '''
        datanum = 0x01
        b = [0]*6
        #包头
        b[0] = 0xEB
        b[1] = 0x90

        #手的id号
        b[2] = self.hand_id

        #数据体长度
        b[3] = datanum

        #指令号
        b[4] = 0xD1

        #校验和
        b[5] = self.checknum(b, 5+datanum)
        
        #向串口发送数据
        putdata = b''
        
        for i in range(1,datanum+6):
            putdata = putdata + self.num2str(b[i-1])
        self.ser.write(putdata)

        # print('读取目标位置发送的数据：%s'%putdata.hex(" "))
        time.sleep(0.5)
        read_num = self.ser.inWaiting()
        getdata = self.ser.read(read_num)
        # print('读取目标位置返回的数据：%s'%getdata.hex(" "))

        Data = getdata.hex(" ").split(" ")
        setpos = [0.0]*6
        for i in range(1,12):
            if i%2 == 0:
                continue
            else:
                s1 = Data[5+i-1]
                s2 = Data[5+i]
                s = s2 + s1
                setpos[int((i-1)/2)] = int(s, 16)
        print('当前位置为：(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)'%(setpos[0], setpos[1], setpos[2], setpos[3], setpos[4], setpos[5]))

    #读取设置角度
    def get_setangle(self):
        datanum = 0x04
        b = [0]*(datanum + 5)
        #包头
        b[0] = 0xEB
        b[1] = 0x90

        #hand_id号
        b[2] = self.hand_id

        #数据个数
        b[3] = datanum
        
        #读操作
        b[4] = 0x11
        
        #地址
        b[5] = 0xCE
        b[6] = 0x05
        
        #读取寄存器的长度
        b[7] = 0x0C
        
        #校验和
        b[8] = self.checknum(b,datanum+4)
        
        #向串口发送数据
        putdata = b''
        
        for i in range(1,datanum+6):
            putdata = putdata + self.num2str(b[i-1])
        self.ser.write(putdata)
        print('发送的数据：')
        for i in range(1,datanum+6):
            print(hex(putdata[i-1]))
        
        getdata= self.ser.read(20)
        print('返回的数据：')
        for i in range(1,21):
            print(hex(getdata[i-1]))
        
        
        setangle = [0]*6
        for i in range(1,7):
            if getdata[i*2+5]== 0xff and getdata[i*2+6]== 0xff:
                setangle[i-1] = -1
            else:
                setangle[i-1] = getdata[i*2+5] + (getdata[i*2+6]<<8)
        return setangle


if __name__=='__main__':
    hand = InspireHandR()
    hand.get_target_position()
    hand.get_current_position()
