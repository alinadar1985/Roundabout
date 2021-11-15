import h5py
import numpy as np
import cv2

ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY), 100]


class DataStorageWriter:

    def __init__(self, file_path, sensor_names=["lidar1", "camera_rgb1", "camera_semseg1", "gnss", "bouding_boxes"]):
        self.h5 = h5py.File(file_path, 'w')
        self.groups = dict()
        for group_name in sensor_names:
            self.groups[group_name] = self.h5.create_group(group_name)
    
    def write_lidar(self, sensor_name, index, lidar_measurement):
        data = []
        for location in lidar_measurement:
            data.append([location.point.x, location.point.y, location.point.z])            
        value = np.array(data).reshape((-1, 3))
        # print("WRITING LIDAR", value.shape)
        self.write_matrix(sensor_name, index, value)
    
    def write_gnss(self, sensor_name, index, gnss_data):        
        value = np.array([[gnss_data.latitude, gnss_data.longitude, gnss_data.altitude]])
        # print("gnss", value)
        self.write_matrix(sensor_name, index, value)
        
    def write_image(self, sensor_name, index, image):
        self.write_matrix(sensor_name, index, image)
    
    def write_rgb_and_compact(self, sensor_name, index, image):
        result, encimg = cv2.imencode('.jpg', image, ENCODE_PARAM)
        self.groups[sensor_name].create_dataset(str(index), (encimg.shape[0], 1), h5py.h5t.NATIVE_UINT8, data=encimg)
        self.groups[sensor_name].attrs["jpeg"] = 1

    def write_collision(self, sensor_name, index, collision_event):
        self.write_matrix(sensor_name, index,
                          np.array([collision_event.normal_impulse.x,
                                    collision_event.normal_impulse.y,
                                    collision_event.normal_impulse.z]).reshape((1, -1)))
    
    def write_vehicles_position(self, vehicles):
        pass

    def write_matrix(self, sensor_name, index, value):
        self.groups[sensor_name].create_dataset(str(index), data=value, maxshape=value.shape, chunks=True)
    
    def set_attributes(self, sensor_name, attrs):
        for param in attrs:
            self.groups[sensor_name].attrs[param] = attrs[param]
    
    def close(self):
        self.h5.close()
        self.h5 = None


class DataStorageReader:

    def __init__(self, file_path):
        self.h5 = h5py.File(file_path, 'r')

        self.sensor_labels = tuple(list(self.h5.keys()))
        assert len(self.sensor_labels) > 0
        
        self.lidars = tuple([label for label in self.sensor_labels if label.startswith('lidar')])
        self.rgb_cameras = tuple([label for label in self.sensor_labels if label.startswith('rgb')])
        self.depth_cameras = tuple([label for label in self.sensor_labels if label.startswith('depth')])
        self.semseg_cameras = tuple([label for label in self.sensor_labels if label.startswith('semseg')])
        self.gnss_sensors = tuple([label for label in self.sensor_labels if label.startswith('gnss')])

    def get_timestamps_as_string(self):
        key = self.rgb_cameras[0]
        return list(self.h5[key].keys())
    
    def get_timestamps(self):
        key = self.sensor_labels[0]
        indexes = list(self.h5[key].keys())
        return [int(x) for x in indexes]

    def get_image(self, sensor_label, index):
        if self.h5[sensor_label].attrs['jpeg'] != 0:
            return cv2.imdecode(self.h5[sensor_label][index][:], 1)
        return self.h5[sensor_label][index][:]

    def get_point_cloud(self, sensor_label, index):
        return self.h5[sensor_label][str(index)][:]
    
    def get_gnss(self, sensor_label, index):
        return self.h5[sensor_label][str(index)][:]
 
    def get_gnss_data(self, sensor_label):
        ts = self.get_timestamps_as_string()
        data = np.array([self.h5[sensor_label][t][:, :] for t in ts]).reshape((-1, 3))
        return data
    
    def get_bounding_boxes(self):
        return self.h5['bounding_box/extent'][:]
    
    def get_bounding_boxes_locations(self):
        return self.h5['bounding_box/location'][:]
    
    def get_vehicle_position(self, index):
        return self.h5['vehicle_position'][str(index)][:]

    def get_sensor_transform(self, sensor_label):
        """
            x, y, z, roll, pitch, yaw
        """
        return self.h5['sensor_transform'][sensor_label][:]

    def get_indexes(self):
        self.h5.keys()
