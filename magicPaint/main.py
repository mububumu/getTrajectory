

import bluetooth
from math import *
import numpy.linalg
from numpy import *
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.font_manager as fm
import json


# x/y/z方向上独立运算
# 以大地坐标系为参照系
# 假设加速度坐标系(A)及方位角坐标系(B)三轴两两相互平行
# 将加速度、方位角转换至大地坐标系下
# poseData[0] : 绕 x 轴逆时针旋转角度
# poseData[1] : 绕 y 轴逆时针旋转角度
# poseData[2] : 绕 z 轴逆时针旋转角度

###################### 变量声明 #######################

# 重力加速度 (m/s^2)
Grav = 9.81

# 姿态数据
# (-180, 180)
poseData = [0, 0, 0]
# 旋转矩阵
RMat = array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])


# 加速度计数据
# 在载体（手机）坐标系下的表示
accelData = [0, 0, 0]

# 之前运动加速度
lastAccel = [0, 0, 0]
# 当前运动加速度（滤除重力加速度后）
currAccel = [0, 0, 0]
# 加速度曲线
accelCurve = []

# 之前速度
lastSpeed = [0, 0, 0]
# 当前速度
currSpeed = [0, 0, 0]
# 速度曲线
speedCurve = []

# 之前位置
lastPos = [0, 0, 0]
# 当前位置
currPos = [0, 0, 0]
# 轨迹
trajectory = []


# 时间控制

# 上次接收到加速度数据的时刻
lastRecvAccelMoment = 0
# 本次接收到加速度数据的时刻
currRecvAccelMoment = 0

# 时间片段
timeSlice = 0
# 时间累积
timeSum = 0

# 等待初始化
waitForInit = 0

# 输出控制
outCtrl = 0

# 轨迹点计数
counter = 0

# JSON文件存储序列
JSONno = 1

###################### 变量声明 #######################


# 姿态角 -> 旋转矩阵
def poseToRotationMat():
    global RMat

    cz = cos(poseData[2])
    sz = sin(poseData[2])
    cy = cos(poseData[1])
    sy = sin(poseData[1])
    cx = cos(poseData[0])
    sx = sin(poseData[0])

    # 基本旋转矩阵
    XM = array([[1, 0, 0],
                [0, cx, sx],
                [0, -sx, cx]])
    YM = array([[cy, 0, sy],
                [0, 1, 0],
                [-sy, 0, cy]])
    ZM = array([[cz, sz, 0],
                [-sz, cz, 0],
                [0, 0, 1]])


    RMat = matmul(matmul(ZM, XM), YM)


# 计算当前运动速度
def calCurrSpeed(delt):
    for i in range(3):
        currSpeed[i] = lastSpeed[i] + (currAccel[i] + lastAccel[i]) / 2 * delt
    print("Current Speed: ", currSpeed[0], currSpeed[1], currSpeed[2])

# 计算当前位置
def calCurrPos(delt):
    for i in range(3):
        currPos[i] = lastPos[i] + lastSpeed[i] * delt + (currAccel[i] + lastAccel[i]) / 4 * delt * delt
    print("Current Position: ", currPos[0], currPos[1], currPos[2])


# 可通过静止状态时为 0 检验
# 是否可以在下位机完成
# 接收的是融合后的姿态，应该重新求旋转矩阵
def GravAccelTrans():
    global currAccel
    global accelData

    accelVector = array(accelData).reshape(3, 1)

    currAccelVec = matmul(RMat, accelVector)
    for i in range(3):
        currAccel[i] = currAccelVec[i][0]

    # 重力加速度分量滤除
    currAccel[2] -= Grav

    print("Size of Acceleration:", numpy.linalg.norm(currAccel))
    print("Current Acceleration: ", currAccel[0], currAccel[1], currAccel[2])

# 刷新
def restart():
    global waitForInit
    global currSpeed
    global currPos
    global trajectory
    global timeSum

    waitForInit = 0
    currSpeed = [0, 0, 0]
    currPos = [0, 0, 0]
    trajectory.clear()
    timeSum = 0

# 绘制轨迹
def outTrajectory():
    global trajectory

    data = array(trajectory)

    x = data[10:, 0]
    y = data[10:, 1]
    z = data[10:, 2]

    # print(x)

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z)

    ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    plt.show()

# 绘制加速度以及速度曲线
def outAccelandSpeed():
    global accelCurve
    global speedCurve

    plt.cla()
    plt.clf()

    d1 = array(accelCurve)
    d2 = array(speedCurve)

    t = d1[10:, 0]
    a = d1[10:, 1]
    v = d2[10:, 1]

    a = plt.plot(t, a, color='red', linewidth=2.0, linestyle='--')
    v = plt.plot(t, v, color='blue', linewidth=3.0, linestyle='-.')

    # my_font = fm.FontProperties(fname="/usr/share/fonts/wqy-microhei/wqy-microhei.ttc")
    # plt.legend(handles=[a, v], labels=["acceleration", "speed"],prop=my_font)

    plt.show()

def saveAsJSON():

    global trajectory
    global JSONno

    trajJSON = json.dumps(trajectory[10:])
    print(trajJSON)

    with open("C:\\Users\\DELL\\Desktop\\DataSource\\" + str(JSONno) + ".json", "w") as f:
        json.dump(trajJSON, f)
        print("加载入文件完成...")

    JSONno += 1

################################ 蓝牙通讯 #####################################
server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", bluetooth.PORT_ANY))
server_sock.listen(1)

port = server_sock.getsockname()[1]
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
bluetooth.advertise_service(server_sock, "SampleServer", service_id=uuid,
                            service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                            profiles=[bluetooth.SERIAL_PORT_PROFILE],
                            # protocols=[bluetooth.OBEX_UUID]
                            )

client_sock, client_info = server_sock.accept()
print("Accepted connection from", client_info)
############################### 蓝牙通讯 #####################################


############################### work #####################################
try:
    while True:
        data = client_sock.recv(1024)
        # if not data:
        #     break
        # print("Received", data)

        # processed data
        data = data.decode()
        pData = data.split(',', -1)

        # 命令类型
        cmdType = pData[0]
        if cmdType == "PD":
            # 接收并转换
            for i in range(3):
                poseData[i] = float(pData[i + 1])
                # Deg -> Rad
                poseData[i] = poseData[i] / 180.0 * pi

            poseToRotationMat()

        elif cmdType == "AD":
            currRecvAccelMoment = time.time()
            # 接收并转换
            for i in range(3):
                accelData[i] = float(pData[i + 1])
            # accelData[0] = -accelData[0]
            GravAccelTrans()

            if waitForInit < 10:
                waitForInit += 1
            else:
                timeSlice = currRecvAccelMoment - lastRecvAccelMoment
                print("timeSlice: ", timeSlice)     # 100 ms
                calCurrSpeed(timeSlice)
                calCurrPos(timeSlice)

            # 状态更新
            timeSum += timeSlice
            lastRecvAccelMoment = currRecvAccelMoment
            lastAccel = currAccel
            lastSpeed = currSpeed
            lastPos = currPos

            # trajectory.append(currPos)

            # 状态存储
            accelCurve.append([timeSum, numpy.linalg.norm(currAccel)])
            speedCurve.append([timeSum, numpy.linalg.norm(currSpeed)])
            trajectory.append([currPos[0], currPos[1], currPos[2]])

        else:
            continue

        print("GET : " + pData[0] + ' ' + pData[1] + ' ' + pData[2] + ' ' + pData[3])

        if outCtrl >= 50:
            outCtrl = 0
            outTrajectory()
            # outAccelandSpeed()

            op = input()
            if op == "y":
                saveAsJSON()

            restart()
        else:
            outCtrl += 1

        print('\n')

except OSError:
    pass
############################### work #####################################

# 关闭蓝牙通讯
client_sock.close()
server_sock.close()

