

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


def draw_rects(img, rects):
    for rect in rects:
        rect = [int(x) for x in rect]
        cv2.rectangle(img, (rect[0], rect[1]), (rect[2], rect[3]), (255, 255, 255), 2)
    return img

def draw_bounding_boxes(image, bounding_boxes):
    for bbox in bounding_boxes:
        points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
        # draw lines
        # base
        image = cv2.line(image, points[0], points[1], (0, 255, 0), 1)
        image = cv2.line(image, points[1], points[2], (0, 255, 0), 1)
        image = cv2.line(image, points[2], points[3], (0, 255, 0), 1)
        image = cv2.line(image, points[3], points[0], (0, 255, 0), 1)
        # top
        image = cv2.line(image, points[4], points[5], (0, 255, 0), 1)
        image = cv2.line(image, points[5], points[6], (0, 255, 0), 1)
        image = cv2.line(image, points[6], points[7], (0, 255, 0), 1)
        image = cv2.line(image, points[7], points[4], (0, 255, 0), 1)
        # base-top
        image = cv2.line(image, points[0], points[4], (0, 255, 0), 1)
        image = cv2.line(image, points[1], points[5], (0, 255, 0), 1)
        image = cv2.line(image, points[2], points[6], (0, 255, 0), 1)
        image = cv2.line(image, points[3], points[7], (0, 255, 0), 1)
    
    return image


def draw_points(image, points, data_colors, thickness=3, distance=None):      
        draw = np.zeros_like(image)
        distance_map = np.zeros_like(image, dtype=np.float)
        
        data_colors //= 2 

        for i in range(points.shape[0]):
            if points[i, 2] > 0:
                cor = (int(data_colors[i, 0]), int(data_colors[i, 1]), int(data_colors[i, 2]))
                x, y = int(1024-points[i, 0]), int(points[i, 1])
                if x >= 1024 or y >= 768 or x < 0 or y < 0:
                    continue

                cv2.circle(draw, (x, y), thickness, cor, thickness)

                print('x', x, 'y', y, 'z', distance[i])
                distance_map[y, x] = 1-distance[i]/2.0
                
        draw2 = draw.astype(np.uint32) + image.astype(np.uint32)
        draw2[draw2 > 255] = 255
        
        return draw2.astype(np.uint8), draw, distance_map


def get_lidar_projected_on_image(points, lidar_transform, camera_transform, vehicle_pos, image, offset_yaw, axis_x, axis_y, thickness=3):
    # LIDAR TO CAR:  LIDAR DATA IS ZERO FRAME
    T1 = rt_matrix(lidar_transform[0], lidar_transform[1], lidar_transform[2], lidar_transform[3]-180, lidar_transform[4], lidar_transform[5]+offset_yaw)
    # LIDAR TO VEHICLE LOCATION
    T2 = rt_matrix(vehicle_pos[0], vehicle_pos[1], vehicle_pos[2], vehicle_pos[3], vehicle_pos[4], vehicle_pos[5])
    # COMPOUDING
    T3 = np.dot(T2, T1)
    extended_pts = np.ones((points.shape[0], 4), dtype=np.float32)
    extended_pts[:, :3] = points.copy()

    # WORLD COORDINATE
    extended_pts = np.dot(T3, extended_pts.T)
    # CAMERA SENSOR TO VEHICLE
    T1 = rt_matrix(camera_transform[0]*axis_x, camera_transform[1]*axis_y, camera_transform[2], camera_transform[3], camera_transform[4], camera_transform[5])
    # SENSOR TO WORLD
    T2 = rt_matrix(vehicle_pos[0], vehicle_pos[1], vehicle_pos[2], vehicle_pos[3], vehicle_pos[4], vehicle_pos[5])
    # COMPOUDING 
    T3 = np.dot(T2, T1)
    # GETTING THE INVERSE TRANSFORMATION
    T4 = np.linalg.inv(T3)

    pts_camera = np.dot(T4, extended_pts)[:3, :]
    calibration = np.identity(3)
    calibration[0, 2] = 1024.0 / 2.0
    calibration[1, 2] = 768.0 / 2.0
    calibration[0, 0] = calibration[1, 1] = 1024 / (2.0 * np.tan(90.0 * np.pi / 360.0))

    cords_y_minus_z_x = np.concatenate([pts_camera[1, :], -pts_camera[2, :], pts_camera[0, :]])
    pts = np.transpose(np.dot(calibration, cords_y_minus_z_x))
    camera_pts = np.concatenate([pts[:, 0] / pts[:, 2], pts[:, 1] / pts[:, 2], pts[:, 2]], axis=1)

    color_map = cv2.applyColorMap(np.arange(0, 256, dtype=np.uint8).reshape((1, -1)), cv2.COLORMAP_RAINBOW).reshape((-1, 3))
    dist = np.linalg.norm(pts_camera.T, axis=1)
    distance = dist.copy()
    t = 200
    dist[dist > t] = t
    dist /= t
    dist *= 255
    pcolors = color_map[dist.astype(np.uint8)]

    #dist_mask = np.linalg.norm(points, axis=1)
    #dist_mask = dist_mask < 2.85
    
    x, y, z = np.abs(points[:, 0]), np.abs(points[:, 1]), points[:, 2]
    print(z.max(), z.min())
    dist_mask = (y < 2.5) * (x < 1) 
    # dist_mask += (z > 1.9)  # filter floor

    distance[dist_mask] = 200
    pcolors[dist_mask] = (0,0,0)
    distance /= 200

    image_lidar, draw, distance_map = draw_points(image.copy(), camera_pts, pcolors, thickness, distance)
    return image_lidar, draw, distance_map


def get_bouding_boxes(reader, camera_transform, index,  offset_yaw, axis_x, axis_y):

    output = []
    rects = []

    # get vehicle position and translate their bouding box to the position
    ego_position = reader.get_vehicle_position(index)[0]
    positions = reader.get_vehicle_position(index)[1:]
    
    bboxes = reader.get_bounding_boxes()[1:]
    bboxes_locations = reader.get_bounding_boxes_locations()[1:]

    print("positions", positions.shape, "\n", positions.astype(np.int16))
    print("boxes    ", bboxes.shape, "\n")
    print("boxes loc", bboxes_locations.shape, "\n", bboxes_locations)

    n = positions.shape[0]
    
    for idx in range(n):
        
        bbox = bboxes[idx]
        bbox_location = bboxes_locations[idx]
        vehicle_pos = positions[idx]

        print('bbox', bbox)
        print('bbox_location', bbox_location)
        print('vehice_pos', vehicle_pos)

        # CUBE TO VEHICLE BOX LOCATION
        T1 = rt_matrix(bbox_location[0], bbox_location[1], bbox_location[2])

        # CUBE TO VEHICLE LOCATION (WCS)
        T2 = rt_matrix(vehicle_pos[0], vehicle_pos[1], vehicle_pos[2],
                       vehicle_pos[3], vehicle_pos[4], vehicle_pos[5])

        bb_box_world = np.dot(np.dot(T2, T1), bbox.T)
        print('box', bb_box_world, bb_box_world.shape)

        # CAMERA SENSOR TO VEHICLE
        print('CAMERA', camera_transform)
        T1 = rt_matrix(camera_transform[0]*axis_x, camera_transform[1]*axis_y, camera_transform[2], camera_transform[3], camera_transform[4], camera_transform[5])

        # SENSOR TO WORLD
        T2 = rt_matrix(ego_position[0], ego_position[1], ego_position[2],
                       ego_position[3], ego_position[4], ego_position[5])

        # COMPOUDING and INVERTIN
        T4 = np.linalg.inv(np.dot(T2, T1))
        bb_box_camera = np.dot(T4, bb_box_world)[:3, :]

        calibration = np.identity(3)
        calibration[0, 2] = 1024.0 / 2.0
        calibration[1, 2] = 768.0 / 2.0
        calibration[0, 0] = calibration[1, 1] = 1024 / (2.0 * np.tan(90.0 * np.pi / 360.0))

        cords_y_minus_z_x = np.concatenate([bb_box_camera[1, :], -bb_box_camera[2, :], bb_box_camera[0, :]])

        bbox = np.transpose(np.dot(calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        
        print("box", camera_bbox.shape, camera_bbox)

        x1, y1 = camera_bbox[:, 0].min(), camera_bbox[:, 1].min()
        x2, y2 = camera_bbox[:, 0].max(), camera_bbox[:, 1].max()

        x1, y1 = max(0, camera_bbox[:, 0].min()), max(0,camera_bbox[:, 1].min())
        x2, y2 = min(1023, camera_bbox[:, 0].max()), min(767,camera_bbox[:, 1].max())
        print('cat', [x1, y1, x2, y2])

        if np.sum(bbox[:, 2] > 0) == bbox[:, 2].shape[0]:
            rects.append([x1, y1, x2, y2])
            output.append(camera_bbox)

    return output, rects


input_file_path = "../dataset/ext/train24.h5"


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
OFFSET_YAW = {'rgb_camera1': 180,
              'rgb_camera2': -90,
              'rgb_camera3': 0,
              'rgb_camera4': 90,
              'rgb_camera5': -180,
              'rgb_camera6': -90,
              'rgb_camera7': 0,
              'rgb_camera8': 90,
              'rgb_camera9': -90,
              'rgb_camera10': 90}

CAM_AXIS = {'rgb_camera1': [1.0, 1.0],
            'rgb_camera2': [1.0, 1.0],
            'rgb_camera3': [1.0, 1.0],
            'rgb_camera4': [-1.0, 1.0],
            'rgb_camera5': [1.0, 1.0],
            'rgb_camera6': [1.0, 1.0],
            'rgb_camera7': [1.0, 1.0],
            'rgb_camera8': [1.0, 1.0],
            'rgb_camera9': [1.0, 1.0],
            'rgb_camera10': [1.0, 1.0]
            }


global enable_point_on_image
enable_point_on_image = False
global draw_bbox
draw_bbox = True

global out_video
out_video = None


def update():
    global COUNTER, enable_point_on_image, draw_bbox, out_video
    idx = indexes[COUNTER]

    points = reader.get_point_cloud(reader.lidars[0], idx)
    lidar_transform = reader.get_sensor_transform(reader.lidars[0])
    ego_pos = reader.get_vehicle_position(idx)[0]

    camera_name = 'rgb_camera10'
    
    camera_transform = reader.get_sensor_transform(camera_name)
    img = reader.get_image(camera_name, idx)
    if enable_point_on_image:
        img, lidar_projected, distance_map = get_lidar_projected_on_image(points, lidar_transform, camera_transform, ego_pos, img,
                                                                          OFFSET_YAW[camera_name], CAM_AXIS[camera_name][0], CAM_AXIS[camera_name][1], 1)
        
        img_d = (distance_map*255.0).astype(np.uint8)
        cv2.imshow('img1', cv2.resize((distance_map*255.0).astype(np.uint8), None, fx=1, fy=1))
        cv2.imshow('img3', cv2.resize(lidar_projected, None, fx=1, fy=1))
        # if out_video is None:
        #     fourcc = cv2.VideoWriter_fourcc(*'XVID')
        #     out_video = cv2.VideoWriter('output8.avi', fourcc, 30.0, (img_d.shape[1], img_d.shape[0]))
        # out_video.write(img_d)

    if draw_bbox:
        boxes, rects = get_bouding_boxes(reader, camera_transform, idx, OFFSET_YAW[camera_name], CAM_AXIS[camera_name][0], CAM_AXIS[camera_name][1])
        img = draw_bounding_boxes(img, boxes)
        img = draw_rects(img, rects)

    QtCore.QCoreApplication.processEvents()

    cv2.imshow('img2', cv2.resize(img, None, fx=1, fy=1))

    if out_video is None:
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out_video = cv2.VideoWriter('output11.avi', fourcc, 30.0, (img.shape[1], img.shape[0]))
    out_video.write(img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        out_video.release()
        exit(1)
    elif key == ord('l'):
        enable_point_on_image = not enable_point_on_image
    elif key == ord('b'):
        draw_bbox = not draw_bbox

    # scatter plot coloring
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

t = QtCore.QTimer()
t.timeout.connect(update)
t.start(30)
QtGui.QApplication.instance().exec_()

