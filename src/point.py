import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



# 读取CSV
df = pd.read_csv("/home/liyq/ws02_robot/src/1.csv")  # 替换为实际路径或改为相对路径

# 可视化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(df['x'], df['y'], df['z'], s=10)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Point Visualization')

plt.show()
