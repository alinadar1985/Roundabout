import open3d as o3d
import numpy as np
import json
import matplotlib.pyplot as plt


#Create two random points
randomPoints = np.random.rand(2, 3)

f = open ('../examples/json_lidar.json', "r")
data = json.loads(f.read())
pointSet = o3d.geometry.PointCloud()
pointSet.points = o3d.utility.Vector3dVector(data)
o3d.visualization.draw_geometries([pointSet])
o3d.io.write_point_cloud("../examples/json_lidar.ply", pointSet)

pcd_load = o3d.io.read_point_cloud("../examples/json_lidar.ply")
o3d.visualization.draw_geometries([pcd_load])










'''
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
# For each set of style and range settings, plot n random points in the box
# defined by x in [23, 32], y in [0, 100], z in [zlow, zhigh].
for i in range(len(data)):
    xs = data[i][0]
    ys = data[i][1]
    zs = data[i][2]
    ax.scatter(xs, ys, zs, marker='o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
'''








'''
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

plt.ion()
fig = plt.figure()
for i in range(50):
    xs = np.random.random(100)*10+20
    ys = np.random.random(100)*5+7
    zs = np.random.random(100)*15+50
   
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xs,ys,zs)
    plt.draw()
    plt.pause(0.0001)
    plt.clf()
'''









'''
np.random.seed(42)

xs = np.random.random(100)*10+20
ys = np.random.random(100)*5+7
zs = np.random.random(100)*15+50

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xs,ys,zs)

plt.show()
'''

