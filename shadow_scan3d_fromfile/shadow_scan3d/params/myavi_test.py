import numpy as np
from mayavi import mlab 


p=np.loadtxt("./points_3d.txt")

x=23*p[:,0].flatten()
y=23*p[:,1].flatten()
z=23*p[:,2].flatten()
"""
s=z
mlab.figure(fgcolor=(1, 1, 1), bgcolor=(0, 0, 0)) #更改背景色
mlab.points3d(x,y,z,s,colormap="copper",mode='point')
mlab.show()
"""
s=np.sqrt(x**2+y**2+z**2)
m=s.mean()
v=s.std()
s=(s-m)/(v)

s=s.reshape((360,600))
mlab.imshow(s,colormap='gist_earth')
mlab.show()