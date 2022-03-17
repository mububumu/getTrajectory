

import bluetooth
from math import *
from numpy import *

# x/y/z方向上独立运算
# 以大地坐标系为参照系（左手系）
# 将加速度转换至大地坐标系下
# poseData[0] : 绕 x 轴逆时针旋转角度
# poseData[1] : 绕 y 轴逆时针旋转角度
# poseData[2] : 绕 z 轴逆时针旋转角度

# 重力加速度 (m/s^2)
GravAccel = -9.8

# 姿态数据
# (-180, 180)
poseData = [0, 0, 0]
# 旋转矩阵
RMat = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

# 加速度计数据
accelData = [0, 0, 0]

# 运动加速度（滤除重力加速度后）
ax = 0
ay = 0
az = 0


# 当前位置
currPos = [0, 0, 0]
# 轨迹
trajectory = []

# 当前速度
currSpeed = [0, 0, 0]

# 时间控制

# 上次接收到加速度数据的时刻
lastRecvAccelMoment = 0
# 本次接收到加速度数据的时刻
currRecvAccelMoment = 0

# 时间片段
timeSlice = 0

# 姿态角 -> 旋转矩阵
def poseToRotationMat():
    cz = cos(poseData[2])
    sz = sin(poseData[2])
    cy = cos(poseData[1])
    sy = sin(poseData[1])
    cx = cos(poseData[0])
    sx = sin(poseData[0])

    RMat[0][0] = cz * cy
    RMat[0][1] = cz * sy * sx - sz * cx
    RMat[0][2] = cz * sy * cx - sz * sx

    RMat[1][0] = sz * cy
    RMat[1][1] = sz * sy * sx + cz * cx
    RMat[1][2] = sz * sy * cx + cz * sx

    RMat[2][0] = -sy
    RMat[2][1] = cy * sx
    RMat[2][2] = cy * cx

# 计算当前运动速度
def calCurrSpeed():
    pass

# 计算当前位置
def calCurrPos():
    pass

# 重力加速度分量滤除
# 可通过静止状态时为0检验
# 是否可以在下位机完成
# 接收的是融合后的姿态，应该重新求旋转矩阵
def GravAccelFiltering():
    filX = RMat[0][2] * GravAccel
    filY = RMat[1][2] * GravAccel
    filZ = RMat[2][2] * GravAccel

    ax = accelData[0] - filX
    ay = accelData[1] - filY
    az = accelData[2] - filZ

    print(ax, ay, az)

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
            poseData[0] = float(pData[1])
            poseData[1] = -float(pData[2])
            poseData[2] = -float(pData[3])

            # Deg -> Rad
            for i in range(3):
                poseData[i] = poseData[i] / 180 * pi
            poseToRotationMat()
        elif cmdType == "AD":
            # 接收并转换
            accelData[0] = float(pData[1])
            accelData[1] = float(pData[2])
            accelData[2] = float(pData[3])

            GravAccelFiltering()
            # calCurrSpeed()
            # calCurrPos()
        else:
            continue

        print("GET : " + pData[0] + ' ' + pData[1] + ' ' + pData[2] + ' ' + pData[3])

except OSError:
    pass


# 关闭蓝牙通讯
client_sock.close()
server_sock.close()

