import serial
import math
import logging
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

#ser = serial.Serial('COM4', baudrate = 230400, timeout = 1)

def transfer_polar_point(distance, angle):
    """
    将极坐标点转换为直角坐标点。
    
    :param distance: 距离
    :param angle: 角度（度数）
    :return: 直角坐标点 (x, y)
    """
    theta_rad = math.radians(angle)
    
    # 计算直角坐标系中的 x 和 y 坐标
    x = distance * math.cos(theta_rad) /1000         #单位mm转m
    y = distance * math.sin(theta_rad) /1000         #单位mm转m
    return x,y



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
    if start_angle <= finally_angle:
        delta_angle = (finally_angle - start_angle) / 16.0
    else:
        delta_angle = (360.0 - start_angle + finally_angle) / 16.0
 
    # 角度原始数据
    angle_string = "起始角度：{:.2f}，结束角度：{:.2f}，偏移角度：{:.2f}".format(start_angle, finally_angle, delta_angle)
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

            print(type(sample_distance),type(sample_angle))
            formatted_string = "第{}个采样点的数据: 距离={}, 角度={:.1f}".format(i+1, sample_distance, sample_angle)
            print(formatted_string)

            x,y = transfer_polar_point(sample_distance, sample_angle)
            cartesian_points.append((x, y))
        else:
            print("第%d个采样点数据无效" % (i+1))

    for item in cartesian_points:
        print(item)
    print(cartesian_points)
    return cartesian_points
    
def read_data(ser, frame_size=60, max_frames=35, header=b'\x55\xAA\x23\x10') -> float:
    """
    从串口读取数据并转换为极坐标点。

    :param ser: 串口对象
    :param frame_size: 帧大小（字节）
    :param max_frames: 最大帧数
    :param header: 帧头校验
    :return: 极坐标点数组
    """
    point_array = np.empty((0, 2), dtype=float)  # 极坐标点缓存

    pkg_count = 0
    while pkg_count < max_frames:
        try:
            count = ser.inWaiting()  # 检查接收缓冲区中的字节数
            #logging.info(f"count is {count}")
            if count >= frame_size:
                tmp_data = ser.read(frame_size)
                if tmp_data[:len(header)] == header:  # 检查帧头和帧内容
                    logging.info(f"Frame {pkg_count + 1} valid header received!")
                    hex_data = ' '.join(f'{byte:02X}' for byte in tmp_data)   # 处理完整的帧
                    logging.info(f"Data: \n{hex_data}")
                    pkg_count += 1
                    cartesian_points = calculate_sample(hex_data)

                    # TODO: 在这里添加数据处理逻辑，比如解析 hex_data 到极坐标点
                    # example: points = parse_data_to_points(tmp_data)
                else:
                    logging.warning("Invalid frame header, ignoring")
                    # 可能需要丢弃当前字节，重新同步
                    ser.read(1)
            #else:
                # 等待缓冲区中有足够的数据
             #   logging.info("Waiting for more data for a complete frame")
        except serial.SerialException as e:
            logging.error(f"Serial error: {e}")
            break

    return cartesian_points

    # 进一步处理 point_array 数据，如果需要
    # TODO: 返回的数据可以根据需求做进一步处理或转换
    #return point_array

def read_serial_data(ser) -> float:
    point_array = np.empty((0, 2), dtype=float)  #极坐标点缓存
    pkg_count = 0
    while pkg_count < 35: # 一般情况下一包数据扫描10度左右,故接收36包数据为一圈
        count = ser.inWaiting() # 检查接收缓冲区中的字节数
        if count > 0:
            tmp_data = ser.read(count)
            tmp_string = ''
            if count < 60:
                tmp_data += ser.read(60 - count) # 偶尔会出现只读了未读满60个字节的情况,此时再读一次
                tmp_string = "小于60"
            elif count > 60:
                ser.read(120 - count) # 偶尔会出现读了超过60个字节的情况,此时清除缓存
                tmp_string = "大于60"
            print_string = "接收缓冲区中的字节数" + tmp_string
            print(print_string) 

            if tmp_data[0] == 0x55 and tmp_data[1] == 0xAA and tmp_data[2] == 0x23 and tmp_data[3] == 0x10: #检查首部 和 包信息 和 采样点数量
            #tmp_data = ser.read(2) #先读取前两个字节
            #if tmp_data == b'\x55\xAA' :  # 检查是否是 55 AA 开头
                # 读取剩余数据
                #remaing_data = ser.read(58)  # 假设帧总共60字节，你需要根据实际情况调整
                #full_data = tmp_data + remaing_data
                # 转换为十六进制
                #hex_data = ' '.join(f'{byte:02X}' for byte in full_data)
                hex_data = tmp_data
                print(f'{hex_data}')

                pkg_count += 1
            # calculate_sample(hex_data)
            else:
               ser.read(1)

def get_serial_data(ser) -> float:
    point_array = np.empty((0, 2), dtype=float)  #极坐标点缓存
    pkg_count = 0
    expect_frame_size = 60 #

    while pkg_count < 35: # 一般情况下一包数据扫描10度左右,故接收36包数据为一圈
        try:
            header_data = ser.read(2)
            if header_data == b'\x55\xAA' :  # 检查是否是 55 AA 开头
                # 读取剩余数据
                remaing_data = ser.read(expect_frame_size - 2)  # 假设帧总共60字节，你需要根据实际情况调整
                if len(remaing_data) == expect_frame_size - 2:
                    full_data = header_data + remaing_data
                    # 转换为十六进制
                    hex_data = ' '.join(f'{byte:02X}' for byte in full_data)
                    pkg_count += 1
                    format_string = "Frame pakeage {}:".format(pkg_count)
                    print(format_string)
                    print(f'{hex_data}')
                else:
                    format_string = "Incomplete frame received, ignoring..."
                    print(format_string)
            #else:
             #   format_string = "Invalid frame header, ignoring..."
              #  print(format_string)
        except serial.SerialException as e:
            format_string = "Serial error: {e}"
            print(format_string)
            break

    # 进一步处理 point_array 数据
    # 这里你可以添加对接收到的极坐标点的处理逻辑
    # 示例：
    # process_point_array(point_array)    

# 
# 雷达数据解析类
class MiniLidar():
    #类初始化
    def __init__(self, port = 'COM4', baudrate = 230400): 
        self.port = port
        self.baudrate = baudrate
        self.ser = None  # 初始化成员变量
        
        try:
            self.ser = serial.Serial(port=self.port, baudrate=self.baudrate)
            if not self.ser.is_open:
                self.ser.open()
            print(f"串口 {self.port} 已成功打开")
        except serial.SerialException as e:
            print(f"串口 {self.port} 打开失败: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"串口 {self.port} 已关闭")

    def is_open(self):
        return self.ser.is_open if self.ser else False

    def get_data(self) -> float:
        points_array = np.empty((0, 2), dtype=float)  #极坐标点缓存
        pkg_count = 0 # 一般情况下一包数据扫描10度左右,故接收36包数据为一圈

        while pkg_count < 35:
            count = self.ser.inWaiting() # 检查接收缓冲区中的字节数
            if count > 0:
                tmp_data = self.ser.read(count)
                if count < 60:
                    tmp_data +=  self.ser.read(60 - count) # 偶尔会出现只读了未读满60个字节的情况,此时再读一次
                elif count > 60:
                    self.ser.read(120 - count)  # 偶尔会出现读了超过60个字节的情况,此时清除缓存

                # 检查数据包的首部 和 包信息 和 采样点数量
                if tmp_data[0] == 0x55 and tmp_data[1] == 0xAA and tmp_data[2] == 0x23 and tmp_data[3] == 0x10:
                    # 旋转速度解析
                    rotate_speed = float(int.from_bytes(tmp_data[4:6],'little')) / 64
                    #print(rotate_speed)
                    #起始角度解析
                    start_angle = float(int.from_bytes(tmp_data[6:8], 'little') - 0xA000) / 64
                    #print(FirstAngle)
                    #结束角度解析
                    finally_angle = float(int.from_bytes(tmp_data[56:58], 'little') - 0xA000) / 64
                    #print(LastAngle)
                    #采样点夹角计算
                    if start_angle > finally_angle:
                        delta_angle = start_angle - finally_angle
                    else:
                        delta_angle = 360.0 - start_angle + finally_angle
                    delta_angle /= 0x10
                    
                    #采样点各角度和距离计算
                    single_package_array = np.zeros([16,2], dtype=float)    #单个包的极坐标点缓存
                    for p_id in range(0,16):
                        #一级角度
                        sample_angle = delta_angle * p_id + start_angle
                        if sample_angle > 360.0:
                            sample_angle -= 360.0
                        single_package_array[p_id, 0] = sample_angle
                        #距离
                        if tmp_data[9 + 3 * p_id] & 0xC0 != 128:    #有效位检测
                            single_package_array[p_id,1] = (int.from_bytes(tmp_data[8+p_id*3:10+p_id*3],'little') & 0x3FFF)
                                #print(PointArray_d[p_id,1])
                        else:
                            single_package_array[p_id, 1] = 0

                    #将当前包采样点数据并入总数据
                    points_array = np.vstack([points_array, single_package_array]) 
                    pkg_count += 1
                else:
                    self.ser.read(1)

        #将极坐标转化为直角坐标
        ang = np.radians(points_array[:,0])   #转弧度制
        dist = points_array[:,1]/1000         #单位mm转m
        x = -np.cos(ang) * dist
        y = np.sin(ang) * dist
        return x,y
        
    def __del__(self):
        self.close()

'''
# test
lidar = MiniLidar() #新建雷达对象
x,y = lidar.get_data() #获得点云坐标
fig, ax = plt.subplots() # 创建一个动画对象
ax.set_aspect('equal', 'box') #限制坐标系比例1:1
x_lim = 4 #限制坐标系范围 4m
y_lim = 4 #限制坐标系范围 4m


def update(i):
    global x,y
    x,y = lidar.get_data()
    ax.clear()
    ax.set_xlim(-x_lim,x_lim)
    ax.set_ylim(-y_lim,y_lim)
    ax.scatter(x,y,s=1)

def main():
    
    # 创建一个动画
    anim = animation.FuncAnimation(fig,update, frames=1, interval=50)

    # 显示动画
    try:
        plt.show() 
    except KeyboardInterrupt:
        plt.close()

    #print(x,y)
        #time.sleep(0.5)
'''
 
if __name__ == "__main__":
    #main()
    x_lim = 4 #限制坐标系范围 4m
    y_lim = 4 #限制坐标系范围 4m

    try:
        ser = serial.Serial('COM4', baudrate = 230400, timeout = 1)
        cartesian_points = read_data(ser)


        def update_frame(frame):
            '''
            plt.clf()  # 清除前一帧的内容
            # 提取当前帧的所有点
            x_data = [point[0] for point in cartesian_points[:frame + 1]]
            y_data = [point[1] for point in cartesian_points[:frame + 1]]

            # 绘制当前帧的点
            plt.scatter(x_data, y_data, color='blue')
            plt.title(f'Frame {frame + 1}')
            plt.xlim(-x_lim, x_lim)  # 根据实际数据范围调整
            plt.ylim(-y_lim, y_lim)  # 根据实际数据范围调整
            plt.xlabel('X Coordinate')
            plt.ylabel('Y Coordinate')
            plt.grid(True)
            '''
            # 提取当前帧的所有点
            x_data = [point[0] for point in cartesian_points[:frame + 1]]
            y_data = [point[1] for point in cartesian_points[:frame + 1]]
            ax.clear()
            ax.set_xlim(-x_lim,x_lim)
            ax.set_ylim(-y_lim,y_lim)
            ax.scatter(x_data, y_data, color='blue')

        # 设置图形
        fig, ax = plt.subplots()
        ax.set_aspect('equal', 'box') #限制坐标系比例1:1
        # 设置动画
        anim = FuncAnimation(fig, update_frame, frames=len(cartesian_points), interval=200)

        # 保存动画
        #anim.save('cartesian_points_animation.mp4', writer='ffmpeg', fps=2)

        # 显示动画
        plt.show()


        #get_serial_data(ser)
        #read_serial_data(ser)
        #    print(f'{hexData}')
    except KeyboardInterrupt:
        ser.close()
        logging.info("串口已关闭")
    