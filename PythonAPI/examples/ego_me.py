import glob
import os
import sys
import time
import json
import csv

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import logging
import random
import time
import numpy as np
import math
import weakref
from queue import Queue
from queue import Empty

from time import gmtime, strftime
from cpm_send import sendCPM


IM_WIDTH = 320
IM_HEIGHT = 240
actor_list =[]
objects = []
data = {}
object_counter = 1

class DetectedObject:
    ID = -1
    x_distance = -1
    y_distance = -1
    x_speed = -1
    y_speed = -1

class EgoDetectors(object):
    objects = None

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        world = client.get_world()
        blueprints = world.get_blueprint_library()
        ego_vehicle = None
        ego_cam = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None
        ID = None
        lat = None
        lon = None
        timestamp =None
        object_counter = 1
        sensor_queue = Queue()
        
        # --------------
        # Spawn ego vehicle
        # --------------
        ego_bp = blueprints.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color','255,0,0')
        ego_bp.set_attribute('role_name','ego')

        carla_map =  world.get_map()
        spawn_points = world.get_map().get_spawn_points()

        '''
        f = open('myFile.csv', 'w', encoding='UTF8')
        writer = csv.writer(f)
        writer.writerow(["X","Y"])
        waypoint_list = carla_map.generate_waypoints(2.0)
        for w in waypoint_list:
            x = w.transform.location.x
            y = w.transform.location.y
            writer.writerow([x,y])
        f.close()
        '''
        

        test_location = carla.Location(0,0,0)
        number_of_spawn_points = len(spawn_points)
       
        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            ego_transform = spawn_points[0]
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            ego_vehicle.set_autopilot(True)
            actor_list.append(ego_vehicle)

            print('\n Ego is spawned and autopilotted')

            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(carla.Location(x=0, y=0, z=100),carla.Rotation(pitch=-90)))    
            #spectator.set_transform(carla.Transform(test_location + carla.Location(z=30),carla.Rotation(pitch=-90)))   
        else: 
            logging.warning('Could not found any spawn points')


        tm = client.get_trafficmanager(8000)
        tm.set_osm_mode(True)

        ego_id = ego_vehicle.id

        # GNSS sensor 
        gnss_bp = blueprints.find('sensor.other.gnss')
        gnss_location = carla.Location(0,0,0)
        gnss_rotation = carla.Rotation(0,0,0)
        gnss_transform = carla.Transform(gnss_location,gnss_rotation)
        gnss_bp.set_attribute("sensor_tick",str(3.0))
        ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(ego_gnss)
        def gnss_callback(gnss):  
            #print("---------------------------------------------------")
            #sendCPM(ID,gnss.latitude,gnss.longitude,gnss.timestamp)
            ego_speed_x = ego_vehicle.get_velocity().x
            ego_speed_y = ego_vehicle.get_velocity().y
            ego_speed = math.sqrt(ego_speed_x*ego_speed_x + ego_speed_y*ego_speed_y) 
            ego_rotation = ego_vehicle.get_transform().rotation
            print("Speed:" + str(ego_speed))
            data['EgoInformation'] = []
            data['EgoInformation'].append({
                    'ID': str(ego_id),
                    'speed': str(ego_speed),
                    'angle': str(ego_rotation.yaw),
                    'latitude': gnss.latitude,
                    'longitude': gnss.longitude
                })
        ego_gnss.listen(lambda gnss: gnss_callback(gnss))

        # collision sensor
        col_bp = blueprints.find('sensor.other.collision')
        col_location = carla.Location(0,0,0)
        col_rotation = carla.Rotation(0,0,0)
        col_transform = carla.Transform(col_location,col_rotation)
        ego_col = world.spawn_actor(col_bp,col_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(ego_col)
        colli_counter = 1
        def col_callback(colli):
            print('---------------------------------------------------\n')
            print("Collision detected: "+str(colli)+'\n')
            print("Collision: " + str(colli.other_actor))
            data['CollisionSensor'] = []
            data['CollisionSensor'].append({
                    'ID': str(colli.other_actor.id),
                    "type": str(colli.other_actor.type_id)
                })
        ego_col.listen(lambda colli: col_callback(colli))
       
        # Obstacle Sensor
        obs_bp = blueprints.find('sensor.other.obstacle')
        obs_bp.set_attribute("only_dynamics",str(True))
        obs_location = carla.Location(0,0,0)
        obs_rotation = carla.Rotation(0,0,0)
        obs_transform = carla.Transform(obs_location,obs_rotation)
        ego_obs = world.spawn_actor(obs_bp,obs_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(ego_obs)
        obs_counter = 1
        def obs_callback(obs):
            # print('---------------------------------------------------\n')
            # print("Obstacle detected:\n"+str(obs)+'\n')
            # print("Distance:\n"+str(obs.distance)+'\n')
            print("obs: " + str(obs.other_actor))
            data['ObstacleSensor'] = []
            data['ObstacleSensor'].append({
                    'ID': str(obs.other_actor.id),
                    "type": str(obs.other_actor.type_id),
                    'distance': obs.distance
                    
                })
        ego_obs.listen(lambda obs: obs_callback(obs))

        while True:
            x=  ego_vehicle.get_transform().location.x
            y=  ego_vehicle.get_transform().location.y
            if y < -100 or y > 100:
                    ego_transform = carla.Transform(carla.Location(x=-x, y=y, z=0),carla.Rotation(pitch=0, yaw=-0, roll=0))
                    ego_vehicle.set_transform(ego_transform)
            if x < -100 or x > 100:
                    ego_transform = carla.Transform(carla.Location(x=x, y=-y, z=0),carla.Rotation(pitch=0, yaw=-0, roll=0))
                    ego_vehicle.set_transform(ego_transform)
                



            world_snapshot = world.wait_for_tick()
            

    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_lane is not None:
                ego_lane.stop()
                ego_lane.destroy()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            if ego_imu is not None:
                ego_imu.stop()
                ego_imu.destroy()
            ego_vehicle.destroy()


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with ego vehicle.')
