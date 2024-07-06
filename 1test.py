import serial
import serial.tools.list_ports
import logging
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class MiniLidar:

    #ser = serial.Serial('COM4', baudrate = 230400, timeout = 1)

    def __init__(self, port = 'COM1', baudrate = 9600): 
        # 初始化串口端口和波特率
        self.port = port
        self.baudrate = baudrate
        self.ser = None  # 初始化成员变量
    
    def open_serial_port(self):
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate)
            if not self.ser.is_open:
                self.ser.open()
            info_string = "串口 {} 已成功打开".format(self.port)
            logging.info(info_string)
        except Exception as e:
            info_string = "串口 {} 打开失败: {}".format(self.port, e)
            logging.warning(info_string)
            self.input_new_port()

    def list_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        print("可用的串口:")
        for port in ports:
            print(f"端口: {port.device}, 描述: {port.description}, 硬件 ID: {port.hwid}")


    def input_new_port(self):
        self.list_serial_ports()
        new_port = input("请输入新的串口编号: ")
        self.port = new_port
        self.open_serial_port()


    def getData(self) -> float:
        PointArray = np.empty((0, 2), dtype=float)
        pkg_count = 0
        while pkg_count < 35:
            count = self.ser.inWaiting()
            if count > 0:
                data = self.ser.read(count)
                if count < 60:
                    data += self.ser.read(60 - count)
                elif count > 60:
                    self.ser.read(120 - count)

                if data[0] == 0x55 and data[1] == 0xAA and data[2] == 0x23 and data[3] == 0x10:
                    RS = float(int.from_bytes(data[4:6], 'little')) / 64
                    FirstAngle = float(int.from_bytes(data[6:8], 'little') - 0xA000) / 64
                    LastAngle = float(int.from_bytes(data[56:58], 'little') - 0xA000) / 64

                    if LastAngle > FirstAngle:
                        delta_Angle = LastAngle - FirstAngle
                    else:
                        delta_Angle = 360.0 - FirstAngle + LastAngle
                    delta_Angle /= 0x10
                    #delta_Angle = 0.625

                    PointArray_d = np.zeros([16, 2], dtype=float)
                    for p_id in range(0, 16):
                        PointArray_d[p_id, 0] = delta_Angle * p_id + FirstAngle

                        if data[9 + 3 * p_id] & 0xC0 != 128:
                            PointArray_d[p_id, 1] = (int.from_bytes(data[8 + p_id * 3:10 + p_id * 3], 'little') & 0x3FFF)
                        else:
                            PointArray_d[p_id, 1] = 0

                    PointArray = np.vstack([PointArray, PointArray_d])
                    pkg_count += 1
                else:
                    self.ser.read(1)

        ang = np.radians(PointArray[:, 0])
        dist = PointArray[:, 1] / 1000
        x = -np.cos(ang) * dist
        y = np.sin(ang) * dist
        return x, y

    def close(self):
        self.ser.close()

def main():
    # 新建雷达对象
    lidarCom = 'COM3'
    lidarBaudrate = 230400
    lidar = MiniLidar(lidarCom, lidarBaudrate)
    lidar.open_serial_port()
    
    # 创建一个动画对象
    fig, ax = plt.subplots()

    # 限制坐标系比例1:1
    ax.set_aspect('equal', 'box')

    # 限制坐标系范围 4m
    xy_limit = 6

    # 累积所有数据点
    all_x, all_y = [], []

    def update(i):
        #global all_x, all_y

        x, y = lidar.getData()
        all_x.extend(x)
        all_y.extend(y)
        ax.clear()
        ax.set_xlim(-xy_limit, xy_limit)
        ax.set_ylim(-xy_limit, xy_limit)
        ax.scatter(all_x, all_y, s=1)
         # 添加网格线
        ax.grid(True, linestyle='--', linewidth=0.5)

    anim = animation.FuncAnimation(fig, update, frames=1, interval=50)
    plt.show()

if __name__ == '__main__':
    main()