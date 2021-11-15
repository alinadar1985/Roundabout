
import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import cv2
import weakref

IM_WIDTH = 320
IM_HEIGHT = 240

#SpawnActor = carla.command.SpawnActor
#SetAutopilot = carla.command.SetAutopilot
#SetVehicleLightState = carla.command.SetVehicleLightState
#FutureActor = carla.command.FutureActor

def process_img(image):
    i = np.array(image.raw_data)  # convert to an array
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))  # was flattened, so we're going to shape it.
    i3 = i2[:, :, :3]  # remove the alpha (basically, remove the 4th index  of every pixel. Converting RGBA to RGB)
    cv2.imshow("", i3)  # show it.
    cv2.waitKey(1)
    return i3/255.0  # normalize

def radar_callback(weak_radar, sensor_data):
    self = weak_radar()
    if not self:
        return
    for detection in sensor_data:
        print('depth: ' + str(detection.depth)) # meters
        print('azimuth: ' + str(detection.azimuth)) # rad
        print('altitude: ' + str(detection.altitude)) # rad
        print('velocity: ' + str(detection.velocity)) # m/s
        detection.join()


actor_list =[]
client = carla.Client('127.0.0.1', 4000)
try:
    client.set_timeout(2.0)
    world = client.get_world()
    blueprints = world.get_blueprint_library()
    bp = blueprints.filter("model3")[0]
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp,spawn_point)
    #vehicle.set_autopilot(True)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    actor_list.append(vehicle)
    camera_bp = blueprints.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', f'{IM_WIDTH}')
    camera_bp.set_attribute('image_size_y', f'{IM_HEIGHT}')
    camera_bp.set_attribute('fov', '110')
    spawn_point = carla.Transform(carla.Location(x=-3.5, z=1.7))
    # spawn the sensor and attach to vehicle.
    sensor_camera = world.spawn_actor(camera_bp, spawn_point, attach_to=vehicle)
    actor_list.append(sensor_camera)
    sensor_camera.listen(lambda data: process_img(data))


    radar_bp = blueprints.filter('sensor.other.radar')[0]
    radar_bp.set_attribute('vertical_fov', '30')  # degrees
    radar_bp.set_attribute('points_per_second', '1500')
    radar_bp.set_attribute('range', '100')  # meters
    sensor_radar = world.spawn_actor(radar_bp, carla.Transform())
    weak_radar = weakref.ref(sensor_radar)
    sensor_radar.listen(lambda sensor_data: radar_callback(weak_radar, sensor_data))

    #time.sleep(5)

finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print("done.")









