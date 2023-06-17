import numpy as np
import matplotlib.pyplot as plt

fileName = "outputdown.txt"
fileName2 = "output.txt"
# 讀取文字檔中的數據
data = np.loadtxt(fileName)
data2 = np.loadtxt(fileName2)
# 將數據拆分為 x, y 和 z 值
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]
x2 = data2[:, 0]
y2 = data2[:, 1]
z2 = data2[:, 2]



# 使用numpy的std函數計算標準差


# 計算位置變化量與索引的關係
position_change = np.sqrt((x[1:] - x[:-1])**2 + (y[1:] - y[:-1])**2)
position_change2 = np.sqrt((x2[1:] - x2[:-1])**2 + (y2[1:] - y2[:-1])**2)
indices = np.arange(len(position_change))
indices2 = np.arange(len(position_change2))
std = np.std(position_change)
std2 = np.std(position_change2)
# 繪製位置變化量與索引的關係圖表
plt.figure(figsize=(8, 6))
plt.plot(indices, position_change)
plt.plot(indices2, position_change2)
plt.xlabel('Time')
plt.ylabel('Position Change')
plt.title('Position Change vs. Time')
plt.grid(True)
plt.show()

# 計算角度變化量與索引的關係
angle_change = np.abs(z[1:] - z[:-1])
angle_change2 = np.abs(z2[1:] - z2[:-1])
indices = np.arange(len(angle_change))
indices2 = np.arange(len(angle_change2))
std3 = np.std(angle_change)
std4 = np.std(angle_change2)
# 繪製角度變化量與索引的關係圖表
plt.figure(figsize=(8, 6))
plt.plot(indices, angle_change)
plt.plot(indices2, angle_change2)
plt.xlabel('Time')
plt.ylabel('Angle Change')
plt.title('Angle Change vs. Time')
plt.grid(True)
plt.show()

print(std*100,std2*100,std3,std4)