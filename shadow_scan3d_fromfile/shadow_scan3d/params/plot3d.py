import numpy as np
from matplotlib import cm
# x, y, z 均为 0 到 1 之间的 100 个随机数
#x = np.array([-0.384,1.449,-0.4878,1.8821,3.8149,1.7782,-12.464])
#y = np.array([-0.8241,-0.711,1.5779,-0.2981,0.0398,2.1039,16.201])
#z = np.array([15.1599,15.1237,17.4266,14.7064,14.43684,16.973,33.164])

points=np.loadtxt("./points.txt")

x=points[:-2,0].flatten()
y=points[:-2,1].flatten()
z=points[:-2,2].flatten()

#平面上点
x2 = np.array([x[0],x[1],x[4],x[3],x[0]])
y2 = np.array([y[0],y[1],y[4],y[3],y[0]])
z2 = np.array([z[0],z[1],z[4],z[3],z[0]])


#铅笔笔尖和笔底
x3 = np.array([x[1],x[2],x[-1]])
y3 = np.array([y[1],y[2],y[-1]])
z3 = np.array([z[1],z[2],z[-1]])

x4 = np.array([x[4],x[5],x[-1]])
y4 = np.array([y[4],y[5],y[-1]])
z4 = np.array([z[4],z[5],z[-1]])

x5 = np.array([x[7],x[8],x[-1]])
y5 = np.array([y[7],y[8],y[-1]])
z5 = np.array([z[7],z[8],z[-1]])

x6 = np.array([x[10],x[11],x[-1]])
y6 = np.array([y[10],y[11],y[-1]])
z6 = np.array([z[10],z[11],z[-1]])

#平面方程
A,B,C,D=points[-2,0],points[-2,1],points[-2,2],points[-1,0]
X=np.arange(-5,5,1)
Y=np.arange(-5,5,1)
X,Y=np.meshgrid(X,Y)
Z=-(A*X+B*Y+D)/C

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


fig = plt.figure()
ax = Axes3D(fig)

#画桌面
surf=ax.plot_trisurf(X.flatten(),Y.flatten(),Z.flatten(),color='w',linewidth=0, antialiased=True)
ax.plot3D(x2,y2,z2)
#画四个视图的铅笔
ax.plot3D(x3,y3,z3,'g',linewidth=0.5)
ax.plot3D(x4,y4,z4,'g',linewidth=0.5)
ax.plot3D(x5,y5,z5,'g',linewidth=0.5)
ax.plot3D(x6,y6,z6,'g',linewidth=0.5)
#画四个视图的投影线
ax.plot3D(np.array([x[0],x[2]]),np.array([y[0],y[2]]),np.array([z[0],z[2]]),'k',linewidth=3)
ax.plot3D(np.array([x[3],x[5]]),np.array([y[3],y[5]]),np.array([z[3],z[5]]),'k',linewidth=3)
ax.plot3D(np.array([x[6],x[8]]),np.array([y[6],y[8]]),np.array([z[6],z[8]]),'k',linewidth=3)
ax.plot3D(np.array([x[9],x[11]]),np.array([y[9],y[11]]),np.array([z[9],z[11]]),'k',linewidth=3)
#画所有点
ax.scatter(x, y, z,s=40,c=['r','r','b']*9+['y'])

#画坐标原点和坐标轴
origin=np.array([0,0,0])
ex=np.array([1,0,0])
xy=np.array([0,1,0])
ez=np.array([0,0,1])
ax.scatter(origin[0],origin[1],origin[2],s=50,c=['k'])
ax.plot3D(np.array([0,5]),np.array([0,0]),np.array([0,0]),'r',linewidth=2)
ax.plot3D(np.array([0,0]),np.array([0,5]),np.array([0,0]),'r',linewidth=2)
ax.plot3D(np.array([0,0]),np.array([0,0]),np.array([0,5]),'c',linewidth=2)
plt.axis('scaled')
plt.axis('equal')
plt.axis('square')
plt.show()


