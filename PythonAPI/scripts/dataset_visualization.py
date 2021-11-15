

# Use Open3D to read the point cloud files and pyqtgraph to show
# OpenCV to read and draw images

# Use PyQt to select and display

# timeline to view the data over time


from utils.data_storage import DataStorageReader
import cv2
import numpy as np
import numpy.linalg as LA
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl



def rt_matrix(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    c_y = np.cos(np.radians(yaw))
    s_y = np.sin(np.radians(yaw))
    c_r = np.cos(np.radians(roll))
    s_r = np.sin(np.radians(roll))
    c_p = np.cos(np.radians(pitch))
    s_p = np.sin(np.radians(pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = x
    matrix[1, 3] = y
    matrix[2, 3] = z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix





input_file_path = "../dataset/data_usecase5.h5"


reader = DataStorageReader(input_file_path)

print('> Datasets available:')
print('>> ', reader.sensor_labels)

indexes = reader.get_timestamps_as_string()

print('>> Total Frames: ', len(indexes))



app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.resize(800, 600)
w.opts['distance'] = 20
w.show()
w.setWindowTitle('LIDAR Point Cloud')

g = gl.GLGridItem()
w.addItem(g)

scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3), dtype=np.float32), color=(0, 1, 0, 0.5), size=0.1, pxMode=False)
scatter.rotate(180, 1, 0, 0)
scatter.translate(0, 0, 2.4)
w.addItem(scatter)

arr = np.arange(0, 256, dtype=np.uint8).reshape((1, -1))
color_map = np.zeros((256, 4), dtype=np.float32)

color = cv2.applyColorMap(arr, cv2.COLORMAP_RAINBOW)
color_map[:, 0:3] = color.astype(np.float) / 255.0
color_map[:, 3] = 0.7


global COUNTER
COUNTER = 0

def update():
    global COUNTER
    idx = indexes[COUNTER]

    img = reader.get_image(reader.rgb_cameras[1], idx)
    cv2.imshow('img', img)
    if cv2.waitKey(1) == ord('q'):
        exit(1)

    points = reader.get_point_cloud(reader.lidars[0], idx)

    colors = np.zeros((points.shape[0], 4))

    dist = LA.norm(points, axis=1)
    dist[dist > 50] = 50
    dist /= 50
    dist *= 255
    colors[:] = color_map[dist.astype(int)]


    scatter.setData(pos=points, color=colors)

    COUNTER += 1
    if len(indexes) <= COUNTER:
        COUNTER = 0
        #exit(1)

t = QtCore.QTimer()
t.timeout.connect(update)
t.start(30)
#QtGui.QApplication.instance().exec_()
QtGui.QApplication.instance().exec()

