import glob
import os
import sys
import time
import json
import csv 
import traci 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
import numpy
import math
import weakref
from geopy.distance import geodesic
import paho.mqtt.client as mqtt

from time import gmtime, strftime
from cpm_send import sendCPM
import threading
threading_condition = threading.Condition()

IM_WIDTH = 320
IM_HEIGHT = 240
SENSOR_ELEV = 0.72
SENSOR_PITCH = 12
actor_list =[]

# Initialize global variables #
data = {}                     #
ego_speed = ego_angle = None  #
ID = lat = lon = None         #
objects = []                  #         
vehicles_front = []           #
vehicles_back = []            #
vehicles_collided = []        #
###############################

object_counter = 1
plot_counter = 1

class DetectedVehicle:
    objects = []
    distance = -1
    speed = -1
    x_distance = -1
    y_distance = -1
    x_speed = -1
    y_speed = -1
    def __init__(self, type):
        type = type


class DetectedObject:
    ID = -1
    distance = -1
    speed = -1
    x_distance = -1
    y_distance = -1
    z_distance = -1
    x_speed = -1
    y_speed = -1
    in_object = False

class EgoDetectors(object):
    objects = None

def spwan_carla(world, map,bp):
    spawn_points = map.get_spawn_points()
    random.shuffle(spawn_points) 
    ego_transform = spawn_points[0]
    #ego_transform = carla.Transform(carla.Location(x=42.11999893, y=-2.039999962, z=3.000000),carla.Rotation(pitch=0.000000, yaw=180, roll=0.000000))
    can_spawn = False
    # Now ego can be spawned
    return world.spawn_actor(bp,ego_transform)

def printDocument(filename,data):
    with open(filename, 'w', encoding='UTF8', newline='') as f:
        n = f.write(str(data))
        f.close()

array_x = []
array_y = []
array_z = []


def run_ego():
    VEHICLES_LIST = ['vehicle.audi.a2',
                 'vehicle.audi.tt',
                 'vehicle.carlamotors.carlacola',
                 'vehicle.citroen.c3',
                 'vehicle.dodge_charger.police',
                 'vehicle.jeep.wrangler_rubicon',
                 #'vehicle.nissan.patrol',
                 #'vehicle.ford.mustang',
                 #'vehicle.bmw.isetta',
                 'vehicle.audi.etron',
                 'vehicle.mercedes-benz.coupe',
                 'vehicle.bmw.grandtourer',
                 'vehicle.toyota.prius',
                 #'vehicle.diamondback.century',
                 'vehicle.tesla.model3',
                 'vehicle.seat.leon',
                 'vehicle.lincoln.mkz2017',
                 'vehicle.volkswagen.t2',
                 'vehicle.nissan.micra',
                 'vehicle.chevrolet.impala'
                ]
    #plt.ion()
    #fig = plt.figure()
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host',metavar='H',default='127.0.0.1',help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port',metavar='P',default=2000,type=int,help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    try:
        world = client.get_world()
        blueprints = world.get_blueprint_library()
        ego_vehicle= ego_cam = ego_col = ego_lane = ego_obs = ego_gnss = ego_imu = None
        ego_bp = blueprints.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color','255,0,0')
        ego_bp.set_attribute('role_name','ego')

        carla_map =  world.get_map()
        #map_waypoints = carla_map.generate_waypoints(2.0)
        ego_vehicle = spwan_carla(world, carla_map,ego_bp)
        print('\n Ego is spawned and autopilotted')
        ego_vehicle.set_autopilot(True)
        spectator = world.get_spectator()
        tm = client.get_trafficmanager(8000) 

        data['ego_ID']= 0
        data['timestamp']= 0
        data['ego_speed'] = 0
        data['ego_angle'] = 0
        data['ego_latitude']= 0
        data['ego_longitude'] = 0 
        data['Radar_front'] = []
        data['Radar_back'] = []
        data['collided'] = []

        # --------------
        # Add Front radar sensor to ego vehicle
        # --------------
        def rad_callback(radar_data, side):
            objects = []
            rad_counter = 1
            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            ego_speed_x = ego_vehicle.get_velocity().x
            ego_speed_y = ego_vehicle.get_velocity().y
            global array_x 
            global array_y 
            global array_z
            array_x =[]
            array_y =[]
            array_z =[]
            for detect in radar_data:
                azi = math.degrees(detect.azimuth)
                alt = math.degrees(detect.altitude)   # + SENSOR_PITCH
                depth =  round(detect.depth,2)
                detected = DetectedObject()
                detected.ID = rad_counter
                detected.distance = round(depth, 2)
                #X = Math.round( dist * ( Math.cos( elev ) * Math.sin( azim ) ) );
                #Y = Math.round( dist * ( Math.cos( elev ) * Math.cos( azim ) ) );
                #Z = Math.round( dist * Math.sin( elev ) ); 
                detected.x_distance = round(depth * math.cos(azi) * math.cos(alt), 2)
                detected.y_distance = round(depth * math.sin(azi) * math.cos(alt), 2)
                detected.z_distance = round(depth * math.sin(alt), 2)
                print("x: " + str(detected.x_distance) + ", y: "+str(detected.y_distance) + " , z: "+str(detected.z_distance))
                print("Depth: "+ str(depth)+ ", Azimuth: "+ str(round(azi,2))+ ",  altitude: "+ str(round(alt,2)))
                print("----------------------------------------------------------------------------------------")
                array_x.append(detected.x_distance)
                array_y.append(detected.y_distance)
                array_z.append(detected.z_distance)

                
                CC = True  
                detected.speed = round(detect.velocity, 2)
                detected.x_speed = round(detect.velocity * math.cos(azi), 2)
                detected.y_speed = round(detect.velocity * math.sin(azi) , 2)    
                if CC == True : # 20 cm above the surface
                    objects.append(detected)
                    fw_vec = carla.Vector3D(x=detect.depth - 0.1)
                    carla.Transform(carla.Location(),carla.Rotation(pitch=current_rot.pitch + alt,yaw=current_rot.yaw + azi,roll=current_rot.roll)).transform(fw_vec)
                    def clamp(min_v, max_v, value):
                        return max(min_v, min(value, max_v))
                    norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                    r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                    g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                    b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                    world.debug.draw_point(radar_data.transform.location + fw_vec,size=0.02,life_time=0.2,persistent_lines=False,color=carla.Color(r, g, b))
                    rad_counter = rad_counter + 1 
            
            array_3D =[]
            for i in range(len(array_x)):
                array_3D.append([array_x[i],array_y[i],array_z[i]])
            
            json_3D = json.dumps(array_3D)
            with open('json_3D.json', 'w') as outfile:
                    n = outfile.write(str(json_3D))
                    outfile.close()

            '''
                if len(array_x) > 100:
                    print("Number of point clouds is: " + str(len(array_x)))
                    #ax = fig.add_subplot(111, projection='3d')
                    for i in range(len(array_x)):
                        plt.scatter(array_x[i], array_y[i], array_z[i])
                        plt.pause(0.05)
                    plt.show()
                    #ax.scatter(array_x,array_y,array_z)
                    #plt.draw()
                    #plt.pause(0.0001)
                    #plt.clf()
            '''

            # ********************************************************* initialize the front radar sensor ***********************************************************
        rad_front_bp = world.get_blueprint_library().find('sensor.other.radar')
        rad_front_bp.set_attribute('horizontal_fov', str(1)) # hotizontal field of view: 60 degrees 
        rad_front_bp.set_attribute('vertical_fov', str(80)) # hotizontal field of view: 30 degrees 
        rad_front_bp.set_attribute('range', str(10)) # 8 meters sensor coverage
        rad_front_bp.set_attribute("sensor_tick",str(2.0)) # initialize the tick interval in 2 seconds
        rad_front_location = carla.Location(x=2.2, z=0.8) # set the location of sensor with 1.8 m in X-axis and 0.8 m in Z-axis
        #rad_front_rotation = carla.Rotation(pitch=0) # rotate sensor 5 degrees in Y-axis
        #rad_front_transform = carla.Transform(rad_front_location,rad_front_rotation)
        rad_front_transform = carla.Transform(rad_front_location)
        rad_front_ego = world.spawn_actor(rad_front_bp,rad_front_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        rad_front_ego.listen(lambda radar_data: rad_callback(radar_data, "Front"))
    
        counter = 0
        while True:
            #threading_condition.acquire()
            spectator_transform =  ego_vehicle.get_transform()
            spectator_transform.location += carla.Location(x = 0, y = 0, z = 10.0)
            ego_yaw = ego_vehicle.get_transform().rotation.yaw
            spectator_transform.rotation = carla.Rotation(pitch = -50, yaw = ego_yaw)
            spectator.set_transform(spectator_transform) 
            counter = counter + 1
            x=  ego_vehicle.get_transform().location.x
            y=  ego_vehicle.get_transform().location.y
            waypoint = carla_map.get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))    
            tm.force_lane_change(ego_vehicle,False)  
            if counter % 20 == 10:    
                CPM_message = {}
                json_data = json.dumps(data)
                json_cpm = json.dumps(CPM_message)
                del json_data
                del json_cpm      
                world_snapshot = world.wait_for_tick() 
                #threading_condition.notify_all()
                #threading_condition.release()
    finally:
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if rad_front_ego is not None:
                rad_front_ego.stop()
                rad_front_ego.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            ego_vehicle.destroy()
            for i in len(actor_list):
                actor_list[i].destroy()

if __name__ == '__main__':

    try:
        run_ego()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with ego vehicle.')









