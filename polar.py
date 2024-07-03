import math
import matplotlib.pyplot as plt

# 定义多个极坐标点的列表
polar_points = [
    (5, 45),   # (r, θ) 第一个点
    (3, 120),  # (r, θ) 第二个点
    (7, 270)   # (r, θ) 第三个点
]

# 转换为直角坐标系中的 x, y 坐标
cartesian_points = []
for r, theta_deg in polar_points:
    # 将角度转换为弧度
    theta_rad = math.radians(theta_deg)
    
    # 计算直角坐标系中的 x 和 y 坐标
    x = r * math.cos(theta_rad)
    y = r * math.sin(theta_rad)
    
    # 将结果添加到列表中
    cartesian_points.append((x, y))

# 绘制散点图
plt.figure(figsize=(8, 6))
for i, (x, y) in enumerate(cartesian_points):
    plt.scatter(x, y, label=f"Point {i+1} ({x:.2f}, {y:.2f})", s=100)

plt.title('Scatter Plot of Polar to Cartesian Coordinates')
plt.xlabel('X')
plt.ylabel('Y')
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.grid(True)
plt.legend()

# 保存图像为文件
plt.savefig('scatter_plot.png')

# 显示图像
plt.show()