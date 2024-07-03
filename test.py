import serial
import math
import logging
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# 雷达数据解析类
class MiniLidar():
    # 类初始化
    def __init__(self, port = 'COM1', baudrate = 9600): 
        # 初始化串口端口和波特率
        self.port = port
        self.baudrate = baudrate
        self.ser = None  # 初始化成员变量
        
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate)
            if not self.ser.is_open:
                self.ser.open()
            info_string = "串口 {} 已成功打开".format(self.port)
            logging.info(info_string)
        except serial.SerialException as e:
            info_string = "串口 {} 打开失败: {e}".format(self.port, e)
            logging.warning(info_string)


    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            info_string = "串口 {} 已关闭".format(self.port)
            logging.info(info_string)

    def is_open(self):
        return self.ser.is_open if self.ser else False

    def get_data(self, frame_size=60, max_frames=35, header=b'\x55\xAA\x23\x10') -> str:
        points_array = np.empty((0, 2), dtype=float)  #极坐标点缓存
        pkg_count = 0 # 一般情况下一包数据扫描10度左右,故接收36包数据为一圈

        while pkg_count < max_frames:
            #self.ser.flushInput()  # 清空输入缓冲区
            try:
                count = self.ser.inWaiting()  # 检查接收缓冲区中的字节数
                #logging.info(f"count is {count}")
                if count >= frame_size:
                    tmp_data = self.ser.read(frame_size)
                    if tmp_data[:len(header)] == header:  # 检查帧头和帧内容
                        logging.info(f"Frame {pkg_count + 1} valid header received!")
                        hex_data = ' '.join(f'{byte:02X}' for byte in tmp_data)   # 处理完整的帧
                        logging.info(f"Data: \n{hex_data}")
                        pkg_count += 1
                        return hex_data
                        #cartesian_points = calculate_sample(hex_data)

                        # TODO: 在这里添加数据处理逻辑，比如解析 hex_data 到极坐标点
                        # example: points = parse_data_to_points(tmp_data)
                    else:
                        logging.warning("Invalid frame header, ignoring")
                        # 可能需要丢弃当前字节，重新同步
                        self.ser.read(1)
                
            #else:
                # 等待缓冲区中有足够的数据
             #   logging.info("Waiting for more data for a complete frame")
            except serial.SerialException as e:
                logging.error(f"Serial error: {e}")
                break

            #return cartesian_points
        '''
        #将极坐标转化为直角坐标
        ang = np.radians(points_array[:,0])   #转弧度制
        dist = points_array[:,1]/1000         #单位mm转m
        x = -np.cos(ang) * dist
        y = np.sin(ang) * dist
        return x,y
        '''
        
    def __del__(self):
        self.close()


def calculate_sample(hex_data):
    """
    从提供的十六进制数据字符串中计算每个采样点的极坐标，并转换为直角坐标。
    
    :param hex_data: 由十六进制字符串组成的数据包
    :return: 直角坐标点列表
    """
    # 将hex_data转换为字节数组以便于解析
    hex_bytes = bytes.fromhex(hex_data)

    # 采样点的角度原始数据
    start_angle = ((hex_bytes[7] << 8 | hex_bytes[6]) - 0xA000) / 64.0
    finally_angle = ((hex_bytes[57] << 8 | hex_bytes[56]) - 0xA000) / 64.0

    # 计算角度偏移
    '''
    if start_angle <= finally_angle:
        delta_angle = (finally_angle - start_angle) / 16.0
    else:
        delta_angle = (360.0 - start_angle + finally_angle) / 16.0
    '''
    # 默认偏移角度0.625
    delta_angle = 0.625
 
    
    # 角度原始数据
    angle_string = "起始角度：{:.2f}，结束角度：{:.2f}，偏移角度：{:.3f}".format(start_angle, finally_angle, delta_angle)
    print(angle_string)

    cartesian_points = []
    # 提取每个采样点数据
    for i in range(16):  # 16个采样点
        offset = 8 + i * 3 # 每个采样点占3字节
        # 采样点数据
        sample_distance = (hex_bytes[offset + 1] & 0x3F) << 8 | hex_bytes[offset]
        valid_sample = (hex_bytes[offset + 1] & 0xC0) == 0x40  # 01xxxxxx (有效标志位)

        if valid_sample:	# 采样点有效
            # 该采样点的角度
            sample_angle = start_angle + delta_angle * i
            if sample_angle > 360.0:
                sample_angle -= 360.0

            #print(type(sample_distance),type(sample_angle))
            formatted_string = "第{}个采样点的数据: 距离={}, 角度={:.1f}".format(i+1, sample_distance, sample_angle)
            logging.info(formatted_string)
            x,y = transfer_polar_point(sample_distance / 1000, sample_angle)
            cartesian_points.append((x, y))
            #return x,y
            
        #else:
         #   info_string = "第{}个采样点数据无效".format(i+1)
          #  logging.info(info_string)

    #for item in cartesian_points:
    #    print(item)
    #print(cartesian_points)
    return cartesian_points

def transfer_polar_point(distance, angle):
    """
    将极坐标点转换为直角坐标点。
    
    :param distance: 距离
    :param angle: 角度（度数）
    :return: 直角坐标点 (x, y)
    """
    theta_rad = math.radians(angle)
    
    # 计算直角坐标系中的 x 和 y 坐标
    x = math.cos(theta_rad) * distance
    y = distance * math.sin(theta_rad)
    return x,y

def main():
    lidarCom = 'COM4'
    lidarBaudrate = 230400
    lidar = MiniLidar(lidarCom, lidarBaudrate)
    if not lidar.is_open():
        print("串口未打开，退出")
        return

    fig, ax = plt.subplots()
    scatter = ax.scatter([], [], s=1)

    ax.set_aspect('equal', 'box')
    xy_limit = 3
    ax.set_xlim(-xy_limit, xy_limit)
    ax.set_ylim(-xy_limit, xy_limit)

    all_x, all_y = [], []

    def update(frame):        
        hexData = lidar.get_data()
        if hexData:
            points = calculate_sample(hexData)
            x, y = zip(*points) if points else ([], [])
            # 将新点追加到现有点上
            all_x.extend(x)
            all_y.extend(y)
            scatter.set_offsets(np.c_[all_x, all_y])
        return scatter,

    ani = FuncAnimation(fig, update, frames=range(100), interval=100, blit=True)
    plt.show()




if __name__ == "__main__":
    main()
    '''
    lidarCom = 'COM4'
    lidarBaudrate = 230400
    lidar = MiniLidar(lidarCom, lidarBaudrate)
    while lidar.is_open():
        print("准备读取数据...")
        hexData = lidar.get_data()
        calculateSample = calculate_sample(hexData)
        for item in calculateSample:
            print(item)


    

    try:
        pass
    except Exception as e:
        print(e)
'''