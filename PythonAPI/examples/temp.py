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
#from cpm_send import sendCPM


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
        file = None
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

        settings = world.get_settings()
        settings.fixed_delta_seconds = 20.0
        world.apply_settings(settings)

        ego_bp = blueprints.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color','255,0,0')
        ego_bp.set_attribute('role_name','ego')

        carla_map =  world.get_map()
        map_waypoints = carla_map.generate_waypoints(2.0)
        spawn_points = world.get_map().get_spawn_points()
        #world.unload_map_layer(carla.MapLayer.Buildings)

        test_location = carla.Location(0,0,0)
        number_of_spawn_points = len(spawn_points)
       
        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            #ego_transform = spawn_points[0]
            #Transform(Location(x=7.603518, y=-43.829613, z=0.275307), Rotation(pitch=0.000000, yaw=-88.586418, roll=0.000000))
            
            ego_transform = carla.Transform(carla.Location(x=2.02, y=70.0, z=0.275307),carla.Rotation(pitch=0.000000, yaw=-90, roll=0.000000))
            print("Spawned here: "+ str(ego_transform))
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            #ego_vehicle.set_autopilot(True)
            actor_list.append(ego_vehicle)
            print('\n Ego is spawned and autopilotted')
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(carla.Location(x=0, y=0, z=120),carla.Rotation(pitch=-90)))  
            #spectator.set_transform(carla.Transform(transform.location + carla.Location(z=150),carla.Rotation(pitch=0)))   
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
        gnss_bp.set_attribute("sensor_tick",str(0.5))
        ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(ego_gnss)
        
        file = open('locations.csv', 'w', encoding='UTF8', newline='')
        writer = csv.writer(file)

        def gnss_callback(gnss):
            #spectator.set_transform(carla.Transform(ego_vehicle.get_transform().location + carla.Location(z=30),carla.Rotation(pitch=-90)))   
            #print("---------------------------------------------------")
            #sendCPM(ID,gnss.latitude,gnss.longitude,gnss.timestamp)
            ego_speed_x = ego_vehicle.get_velocity().x
            ego_speed_y = ego_vehicle.get_velocity().y
            ego_speed = math.sqrt(ego_speed_x*ego_speed_x + ego_speed_y*ego_speed_y) 
            ego_rotation = ego_vehicle.get_transform().rotation
            #print("Speed:" + str(int(np.around(ego_speed,2))))
            #print("--> " + str(ego_vehicle.get_transform()))
            #writer.writerow([ego_vehicle.get_location().x,ego_vehicle.get_location().y])
            # data['EgoInformation'] = []
            # data['EgoInformation'].append({
            #         'ID': str(ego_id),
            #         'speed': str(int(numpy.around(ego_speed,2))),
            #         'angle': str(ego_rotation.yaw),
            #         'latitude': gnss.latitude,
            #         'longitude': gnss.longitude
            #     })
        ego_gnss.listen(lambda gnss: gnss_callback(gnss))
      
        
        counter = 0
        #ego_vehicle.set_simulate_physics(False)
        queue = []
        #waypoint = world.get_map().get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving)) 
        point = None
        while True:
            ego_vehicle.set_transform(waypoint.transform)   

            waypoint = carla_map.get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
            if point == None:
                nexts = waypoint.next(3.0)
            else:
                nexts = point.next(3.0)

            real_next = None
            old_position = -1000000
            point = nexts[0]

            if len(nexts) > 1:
                location1 = nexts[0].transform.location
                location2 = nexts[1].transform.location

                center = [0,0]
                p1 = [location1.x,location1.y]
                p2 = [location2.x,location2.y]

                distance1 = math.dist(center, p1)
                distance2 = math.dist(center, p2)
                print("distance1: "+str(distance1))
                print("distance2: "+str(distance2))
                print("--------------------------------------")

                if distance1 > distance2:
                    ego_vehicle.set_transform(nexts[1].transform)
                else:
                    ego_vehicle.set_transform(nexts[0].transform)


           
            x=  ego_vehicle.get_transform().location.x
            y=  ego_vehicle.get_transform().location.y
            if y > 75:
                ego_transform = carla.Transform(carla.Location(x=6.0, y=y-5, z=0.275307),carla.Rotation(pitch=0.000000, yaw=-90, roll=0.000000))
                ego_vehicle.set_transform(ego_transform)
            if y < -75:
                ego_transform = carla.Transform(carla.Location(x=6.0, y=y-5, z=0.275307),carla.Rotation(pitch=0.000000, yaw=-90, roll=0.000000))
                ego_vehicle.set_transform(ego_transform)
            if x > 75:
                ego_transform = carla.Transform(carla.Location(x=6.0, y=y-5, z=0.275307),carla.Rotation(pitch=0.000000, yaw=-90, roll=0.000000))
                ego_vehicle.set_transform(ego_transform)
            if x < -60:
                ego_transform = carla.Transform(carla.Location(x=6.0, y=y-5, z=0.275307),carla.Rotation(pitch=0.000000, yaw=-90, roll=0.000000))
                ego_vehicle.set_transform(ego_transform)
            
            time.sleep(0.1)
            world.tick() 

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
