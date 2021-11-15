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
#from cpm_send import sendCPM
from mongo import saveMongo


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



def main():
    #traci.init(8813)
    #traci.setOrder(2)
    plt.ion()
    fig = plt.figure()
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
        timestamp = None

        ego_bp = blueprints.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color','255,0,0')
        ego_bp.set_attribute('role_name','ego')

        carla_map =  world.get_map()
        #map_waypoints = carla_map.generate_waypoints(2.0)
        ego_vehicle = spwan_carla(world, carla_map,ego_bp)
        print('\n Ego is spawned and autopilotted')
        ego_vehicle.set_autopilot(True)
        
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=0, y=0, z=120),carla.Rotation(pitch=-90)))  
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

        # collision sensor
        def col_callback(colli):       
            collided_actor = colli.other_actor
            print('---------------------------------------------------\n')
            print("Collision detected: "+str(colli)+'\n')
            print("Collision: " + str(colli.other_actor))
            collided = DetectedVehicle("collided")
            collided.distance = 0
            collided.x_distance = 0
            collided.y_distance = 0
            
            collided.speed = collided_actor.get_velocity()
            collided.x_speed = colli.other_actor.get_velocity().x
            collided.y_speed = colli.other_actor.get_velocity().y
            vehicles_collided.append(collided)  
              # collision info
            data['collided'] = []
            for v_collided in vehicles_collided:
                data['collided'].append({
                    'distance': v_collided.distance,
                    'speed': v_collided.speed,
                    'x_distance': v_collided.x_distance,
                    'y_distance': v_collided.y_distance,
                    'x_speed': v_collided.x_speed,
                    'y_speed': v_collided.y_speed,
                })   
       
        col_bp = blueprints.find('sensor.other.collision')
        col_location = carla.Location(0,0,0)
        col_rotation = carla.Rotation(0,0,0)
        col_transform = carla.Transform(col_location,col_rotation)
        ego_col = world.spawn_actor(col_bp,col_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(ego_col)
        ego_col.listen(lambda colli: col_callback(colli))
       
        # --------------
        # Add Front & Back radar sensors to ego vehicle
        # --------------
        def rad_callback(radar_data, side):
            objects = []
            array_x = []
            array_y = []
            array_z = []
            rad_counter = 1
            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            ego_speed_x = ego_vehicle.get_velocity().x
            ego_speed_y = ego_vehicle.get_velocity().y
            for detect in radar_data:
                azi = math.degrees(detect.azimuth)
                alt = math.degrees(detect.altitude) + SENSOR_PITCH
                depth =  detect.depth
                detected = DetectedObject()
                detected.ID = rad_counter
                detected.distance = round(depth, 2)
                detected.x_distance = round(depth * math.cos(azi), 2)
                detected.y_distance = round(depth * math.sin(azi), 2)
                detected.z_distance = round(depth * math.sin(alt), 2)
                if side == "Front":
                    array_x.append(detected.x_distance)
                    array_y.append(detected.y_distance)
                    array_z.append(detected.z_distance)

                detected.speed = round(detect.velocity, 2)
                detected.x_speed = round(detect.velocity * math.cos(azi), 2)
                detected.y_speed = round(detect.velocity * math.sin(azi) , 2)    
                if detected.z_distance > -SENSOR_ELEV + 0.2 : # 20 cm above the surface
                    objects.append(detected)
                    fw_vec = carla.Vector3D(x=detect.depth - 0.1)
                    carla.Transform(carla.Location(),carla.Rotation(pitch=current_rot.pitch + alt,yaw=current_rot.yaw + azi,roll=current_rot.roll)).transform(fw_vec)
                    def clamp(min_v, max_v, value):
                        return max(min_v, min(value, max_v))
                    norm_velocity = detect.velocity / velocity_range # range [-1, 1]
                    r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
                    g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
                    b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                    world.debug.draw_point(radar_data.transform.location + fw_vec,size=0.075,life_time=0.1,persistent_lines=False,color=carla.Color(r, g, b))
                    rad_counter = rad_counter + 1 

            if len(array_x) > 100:
                print("Number of point clouds is: " + str(len(array_x)))
                ax = fig.add_subplot(111, projection='3d')
                ax.scatter(array_x,array_y,array_z)
                plt.draw()
                plt.pause(0.0001)
                plt.clf()

            array_object = []
            shapes = []
            not_shaped = [x for x in objects  if x.in_object == False]
            not_shaped = sorted(not_shaped, key=lambda x: x.distance)

            while len(not_shaped) > 0:
                not_shaped = [x for x in not_shaped if x.in_object == False]
                not_shaped = sorted(not_shaped, key=lambda x: x.distance)
                array_object = []
                shape = DetectedVehicle(side)
                not_shaped[0].in_object = True
                # set the nearest object as root
                root = not_shaped[0]
                array_object.append(root)
                object_left = None
                object_right = None
                for i in range(1, len(not_shaped)):
                    if not_shaped[i].y_distance < root.y_distance:
                    ######################################  find the left object ################################################
                        if object_left == None:
                            distance = math.sqrt((not_shaped[i].x_distance - root.x_distance) **2 +( not_shaped[i].y_distance - root.y_distance) **2 )
                        else:
                            distance = math.sqrt((not_shaped[i].x_distance - object_left.x_distance) **2 +( not_shaped[i].y_distance - object_left.y_distance) **2 )
                    
                        if distance < 5 and abs(not_shaped[i].speed - root.speed) < 2.0 :
                            not_shaped[i].in_object = True # tag the selected object as "in_object"
                            array_object.append(not_shaped[i])
                            object_left = not_shaped[i]
                    
                    elif not_shaped[i].y_distance > root.y_distance:
                    ######################################  find the right object ################################################
                        if object_right == None:
                            distance = math.sqrt((not_shaped[i].x_distance - root.x_distance) **2 +( not_shaped[i].y_distance - root.y_distance) **2 )
                        else:
                            distance = math.sqrt((not_shaped[i].x_distance - object_right.x_distance) **2 +( not_shaped[i].y_distance - object_right.y_distance) **2 )
                        
                        if distance < 5 and abs(not_shaped[i].speed - root.speed) < 2.0 :
                            not_shaped[i].in_object = True  # tag the selected object as "in_object"
                            array_object.append(not_shaped[i])
                            object_right = not_shaped[i]
                
                shape.objects = array_object
                shape.distance = root.distance
                shape.x_distance = root.x_distance
                shape.y_distance = root.y_distance
                shape.speed =  root.speed
                shape.x_speed = root.x_speed
                shape.y_speed = root.y_speed
                
                if len(shape.objects) > 9: 
                    shapes.append(shape)
                    print(side + " vehicle: << objects: " + str(len(shape.objects)) + "  Distance: " + str(round(shape.distance,2)) + " m   Speed: " + str(round(shape.speed,2)) + " m/s >>")
               
                not_shaped = [x for x in not_shaped if x.in_object == False]
            # end of while not shaped
            if side == "Front":
                world.debug.draw_string(radar_data.transform.location, side + " : " + str(len(shapes)) + " vehs.", False,  carla.Color(100,100,0), 1) 
                for v_front in shapes:
                    data['Radar_front'].append({
                        'distance': v_front.distance,
                        'speed': v_front.speed,
                        'x_distance': v_front.x_distance,
                        'y_distance': v_front.y_distance,
                        'x_speed': v_front.x_speed,
                        'y_speed': v_front.y_speed,
                        })
            else: #Side Back
                world.debug.draw_string(radar_data.transform.location + carla.Location(x = -10), side + " : " + str(len(shapes)) + " vehs.", False,  carla.Color(100,100,0), 1)
                for v_back in shapes:
                    data['Radar_back'].append({
                        'distance': v_back.distance,
                        'speed': v_back.speed,
                        'x_distance': v_back.x_distance,
                        'y_distance': v_back.y_distance,
                        'x_speed': v_back.x_speed,
                        'y_speed': v_back.y_speed,
                        })

        # ********************************************************* initialize the front radar sensor ***********************************************************
        rad_front_bp = world.get_blueprint_library().find('sensor.other.radar')
        rad_front_bp.set_attribute('horizontal_fov', str(60)) # hotizontal field of view: 60 degrees 
        rad_front_bp.set_attribute('vertical_fov', str(30)) # hotizontal field of view: 30 degrees 
        rad_front_bp.set_attribute('range', str(10)) # 8 meters sensor coverage
        rad_front_bp.set_attribute("sensor_tick",str(2.0)) # initialize the tick interval in 2 seconds
        # tesla M3 { lenght: 4.694 m, width: 1.85 m, height: 1.443 m }
        # Sensor location :{x-axis: 2.347 m, y-axis: 0, z-axis: 0.72 m}
        rad_front_location = carla.Location(x=2.347, z=SENSOR_ELEV)
        rad_front_rotation = carla.Rotation(pitch=SENSOR_PITCH) # rotate sensor 12 degrees in Y-axis
        rad_front_transform = carla.Transform(rad_front_location,rad_front_rotation)
        rad_front_ego = world.spawn_actor(rad_front_bp,rad_front_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        rad_front_ego.listen(lambda radar_data: rad_callback(radar_data, "Front"))
        # ********************************************************* initialize the back radar sensor ***********************************************************
        rad_back_bp = world.get_blueprint_library().find('sensor.other.radar')
        rad_back_bp.set_attribute('horizontal_fov', str(60)) # hotizontal field of view: 60 degrees 
        rad_back_bp.set_attribute('vertical_fov', str(30)) # hotizontal field of view: 30 degrees 
        rad_back_bp.set_attribute('range', str(10)) # 8 meters sensor coverage
        rad_back_bp.set_attribute("sensor_tick",str(2.0))  # initialize the tick interval in 2 seconds
        rad_back_location = carla.Location(x=-2.347, z=SENSOR_ELEV) 
        rad_back_rotation = carla.Rotation(pitch=SENSOR_PITCH, yaw= 180) # rotate sensor 170 degrees in Y-axis
        rad_back_transform = carla.Transform(rad_back_location,rad_back_rotation)
        rad_back_ego = world.spawn_actor(rad_back_bp,rad_back_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        rad_back_ego.listen(lambda radar_data: rad_callback(radar_data, "Back"))

        # GNSS sensor 
        gnss_bp = blueprints.find('sensor.other.gnss')
        gnss_bp.set_attribute("sensor_tick",str(0.5))
        gnss_location = carla.Location(0,0,0)
        gnss_rotation = carla.Rotation(0,0,0)
        gnss_transform = carla.Transform(gnss_location,gnss_rotation)
        ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(ego_gnss)
        def gnss_callback(gnss):
            data['ego_ID']= 1
            data['ego_latitude']= gnss.latitude
            data['ego_longitude'] = gnss.longitude    
            data['timestamp'] =  gnss.timestamp
            ego_speed_x = ego_vehicle.get_velocity().x
            ego_speed_y = ego_vehicle.get_velocity().y
            ego_speed = math.sqrt(ego_speed_x*ego_speed_x + ego_speed_y*ego_speed_y) 
            data['ego_speed'] = round(ego_speed,2)
            data['ego_angle'] =  round(ego_vehicle.get_transform().rotation.yaw,2)
            
        ego_gnss.listen(lambda gnss: gnss_callback(gnss))
        
        counter = 0
        while True:
            spectator_transform =  ego_vehicle.get_transform()
            spectator_transform.location += carla.Location(x = -20, z = 15.0)
            spectator_transform.rotation = carla.Rotation(pitch = -35)
            spectator.set_transform(spectator_transform) 
            counter = counter + 1
            x=  ego_vehicle.get_transform().location.x
            y=  ego_vehicle.get_transform().location.y
            waypoint = carla_map.get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))    
            tm.force_lane_change(ego_vehicle,False)  

            if counter % 20 == 10:
                object_id = 0
                perceived_object_container=[]
                mixer = data['Radar_front'] + data['Radar_back'] + data['collided']
                print("---> Front detected:    " + str(len(data['Radar_front'])) )
                print("---> Back detected:     " + str(len(data['Radar_back'])))   
                print("---> collision: " + str(len(data['collided'])) ) 
                data['Radar_front'] = []
                data['Radar_back'] = []
                data['collided'] = []
                for i in range(len(mixer)):
                    object_id =  object_id + 1
                    perceived_object_container.append({
                        "perceived_object":{
                            "object_id": object_id,
                            "time_of_measurement": 1,
                            "object_confidence": 0,
                            "x_distance": mixer[i]['x_distance'],
                            "y_distance": mixer[i]['y_distance'],
                            "x_speed": mixer[i]['x_speed'],
                            "y_speed": mixer[i]['y_speed'],
                            "confidence": {
                                "x_distance": 2,
                                "y_distance": 2,
                                "x_speed": 3,
                                "y_speed": 3
                                },      
                            "object_reference_point": 0
                        }}) 
                CPM_message = {
                    "type": "cpm",
                    "origin": "EURECOM_Geoserver_Client",
                    "version": "1.0.0",
                    "source_uuid": data['ego_ID'],
                    "timestamp": data['timestamp'],
                    "message": {
                        "protocol_version": 1,
                        "station_id": float(data['ego_ID']),
                        "generation_delta_time": 1,
                        "management_container": {
                            "station_type": 5,
                            "reference_position": {
                                "latitude": int(numpy.around(data['ego_latitude'],7)*10000000),
                                "longitude": int(numpy.around(data['ego_longitude'],7)*10000000),
                                "altitude": 0},
                            "confidence": {
                                "position_confidence_ellipse": {
                                    "semi_major_confidence": 100,
                                    "semi_minor_confidence": 50,
                                    "semi_major_orientation": 180
                                },
                            "altitude": 3
                            }
                        },
                        "originating_vehicle_container": {
                            "heading": data['ego_angle'],
                            "speed": data['ego_speed'] ,
                            "drive_direction": 0,
                            "confidence": {
                                "heading": 2,
                                "speed": 3
                            } 
                        },
                        "perceived_object_container": perceived_object_container
                    }
                }

                json_data = json.dumps(data)
                json_cpm = json.dumps(CPM_message)
                # publish the CPM message in the EURECOM server with its specific topic
                #client = mqtt.Client(protocol=mqtt.MQTTv31)
                #client.connect("5gcroco.eurecom.fr")
                #client.publish("eurecom/cpm/message",json_cpm)

                #if len(perceived_object_container) > 0:
                #    saveMongo(CPM_message)

                print(".............................................................................................")
                print("...................................... Sent CPM Done ........................................")
                print(".............................................................................................")
                
                # write CPM file and save it externally for debugging purpose
                with open('cpm_new.json', 'w') as outfile:
                    n = outfile.write(str(json_cpm))
                    outfile.close()
                
                # write the data structure and save it externally for debugging purpose
                with open('data_new.json', 'w') as outfile:
                    n = outfile.write(str(json_data))
                    outfile.close()

                del json_data
                del json_cpm
                
            world_snapshot = world.wait_for_tick() 

                   

    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if rad_front_ego is not None:
                rad_front_ego.stop()
                rad_front_ego.destroy()
            if rad_back_ego is not None:
                rad_back_ego.stop()
                rad_back_ego.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            ego_vehicle.destroy()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n Done with ego vehicle.')







