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

    client = carla.Client(args.host, args.port)#"10.188.148.6"
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

        #settings = world.get_settings()
        #settings.fixed_delta_seconds = 0.15
        #settings.synchronous_mode = True
        #world.apply_settings(settings)

        ego_bp = blueprints.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color','255,0,0')
        ego_bp.set_attribute('role_name','ego')

        carla_map =  world.get_map()
        map_waypoints = carla_map.generate_waypoints(2.0)
        spawn_points = world.get_map().get_spawn_points()
        #world.unload_map_layer(carla.MapLayer.Buildings)

        test_location = carla.Location(0,0,0)
        number_of_spawn_points = len(spawn_points)
        waypoints = carla_map.generate_waypoints(15)
 
        if 0 < number_of_spawn_points:
            random.shuffle(spawn_points)
            #ego_transform = spawn_points[0]
            #Transform(Location(x=7.603518, y=-43.829613, z=0.275307), Rotation(pitch=0.000000, yaw=-88.586418, roll=0.000000))
            
            ego_transform = carla.Transform(carla.Location(x=2.02, y=45.0, z=0.275307),carla.Rotation(pitch=0.000000, yaw=-90, roll=0.000000))
            print("Spawned here: "+ str(ego_transform))
            ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
            ego_vehicle.set_autopilot(True)
            actor_list.append(ego_vehicle)
            print('\n Ego is spawned and autopilotted')
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(carla.Location(x=0, y=0, z=60),carla.Rotation(pitch=-90)))  
            #spectator.set_transform(carla.Transform(transform.location + carla.Location(z=150),carla.Rotation(pitch=0)))   
            #spectator.set_transform(carla.Transform(test_location + carla.Location(z=30),carla.Rotation(pitch=-90)))   
        else: 
            logging.warning('Could not found any spawn points')

        ego_id = ego_vehicle.id

        # GNSS sensor 
        gnss_bp = blueprints.find('sensor.other.gnss')
        gnss_location = carla.Location(0,0,0)
        gnss_rotation = carla.Rotation(0,0,0)
        gnss_transform = carla.Transform(gnss_location,gnss_rotation)
        gnss_bp.set_attribute("sensor_tick",str(0.5))
        ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(ego_gnss)
        

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
      
        
        def firstBetter(wp1,wp2):
            #world.world.debug.draw_string(wp1.transform.location, 'O', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=5.0,persistent_lines=True)
            #world.world.debug.draw_string(wp2.transform.location, 'O', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=5.0,persistent_lines=True)
            nexts1 = wp1.next(3)
            nexts2 = wp2.next(3)
            distance1 = 0
            distance2 = 0
            #print("...................")
            #world.world.debug.draw_string(nexts1[0].transform.location, 'O', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=5.0,persistent_lines=True)
            #world.world.debug.draw_string(nexts2[0].transform.location, 'O', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=5.0,persistent_lines=True)

            location1_1 = nexts1[0].transform.location
            p1_1 = [location1_1.x,location1_1.y]
            distance1 = math.sqrt( ((p1_1[0])**2)+((p1_1[1])**2))
            if len(nexts1) > 1:
                #world.world.debug.draw_string(nexts1[1].transform.location, 'O', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=5.0,persistent_lines=True)
                location1_2 = nexts1[1].transform.location
                p1_2 = [location1_2.x,location1_2.y]
                distance1_2 = math.sqrt( ((p1_2[0])**2)+((p1_2[1])**2))
                distance1 = (distance1 + distance1_2)/2
            
            location2_1 = nexts2[0].transform.location
            p2_1 = [location2_1.x,location2_1.y]
            distance2 = math.sqrt( ((p2_1[0])**2)+((p2_1[1])**2))
            if len(nexts2) > 1:
                #world.world.debug.draw_string(nexts2[1].transform.location, 'O', draw_shadow=False,color=carla.Color(r=255, g=0, b=0), life_time=5.0,persistent_lines=True)
                location2_2 = nexts2[1].transform.location
                p2_2 = [location2_2.x,location2_2.y]
                distance2_2 = math.sqrt( ((p2_2[0])**2)+((p2_2[1])**2))
                distance2 = (distance2 + distance2_2)/2

            print("distance1: "+str(distance1))
            print("distance2: "+str(distance2))
            if distance1 < distance2:
                return True
            else:
                return False

        points = []
        counter = 0
        #ego_vehicle.set_simulate_physics(False)
        #queue = []
        waypoint = world.get_map().get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving)) 
        next_point = None
        iteration = 10
        while True:   
            waypoint = carla_map.get_waypoint(ego_vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
            angle = math.radians(-4)
            rotate = -4
           
            nexts = waypoint.next(3)
            if len(nexts) > 1: # one waypoint in front of vehicle
                print(str(len(nexts))  +"  options...")
                if firstBetter(nexts[0],nexts[1]):
                    next_point = nexts[0]
                else:
                    next_point = nexts[1]
                ego_vehicle.set_transform(next_point.transform)
                
                left = next_point.get_left_lane()
                ego_vehicle.set_transform(left.transform)

                points = next_point.next_until_lane_end(2.0)
                count = len(points)
                print(str(count))
                if count > 20:
                    ego_speed_x = ego_vehicle.get_velocity().x
                    ego_speed_y = ego_vehicle.get_velocity().y
                    ego_speed = math.sqrt(ego_speed_x*ego_speed_x + ego_speed_y*ego_speed_y) 
                    print("My speed is : " + str(ego_speed))
                    c = 0
                    while c < count:
                        print(str(points[c].transform.location))
                        ego_vehicle.set_transform(points[c].transform)
                        c = c+1
                        time.sleep(0.1)
                        bbb = world.wait_for_tick()
                        if c == count:
                            c=0
                else:
                    #ego_vehicle.set_wheel_steer_direction(carla.VehicleWheelLocation.FR_Wheel, 22.0)
                    #ego_vehicle.set_wheel_steer_direction(carla.VehicleWheelLocation.FL_Wheel, 22.0)
                    velocity = ego_vehicle.get_velocity()
                    old_location = ego_vehicle.get_transform().location
                    new_x = math.cos(angle) * (old_location.x) - math.sin(angle) * (old_location.y )
                    new_y =  math.sin(angle) * (old_location.x) + math.cos(angle) * (old_location.y)
                    new_location = carla.Location(x = new_x, y = new_y, z = 0.275307)
                    old_rotation = ego_vehicle.get_transform().rotation
                    new_rotation = carla.Rotation(pitch=old_rotation.pitch, yaw=old_rotation.yaw+rotate, roll=old_rotation.roll) # next_point.transform.rotation # + carla.Rotation(yaw=next_point.transform.rotation.yaw-5)
                    new_transform = carla.Transform(new_location,new_rotation)
                    ego_vehicle.set_transform(new_transform)
                    iteration = 0

                '''
                 elif iteration < 10:
                 angle1 = math.radians(-3)
                 rotate1 = -3
                 old_location = ego_vehicle.get_transform().location
                 new_x = math.cos(angle1) * (old_location.x) - math.sin(angle1) * (old_location.y )
                 new_y =  math.sin(angle1) * (old_location.x) + math.cos(angle1) * (old_location.y)
                 new_location = carla.Location(x = new_x, y = new_y, z = 0.275307)
                 old_rotation = ego_vehicle.get_transform().rotation
                 new_rotation = carla.Rotation(pitch=old_rotation.pitch, yaw=old_rotation.yaw+rotate1, roll=old_rotation.roll) # next_point.transform.rotation # + carla.Rotation(yaw=next_point.transform.rotation.yaw-5)
                 new_transform = carla.Transform(new_location,new_rotation)
                 ego_vehicle.set_transform(new_transform)
                 iteration = iteration + 1
                 print("......"+ str(iteration) +".......")
                ''' 
            else:
                location = ego_vehicle.get_transform().location
                rotation = ego_vehicle.get_transform().rotation
                if location.x > 0 and location.x < 25 and location.y > 0 and location.y < 25 and rotation.yaw > -50 and rotation.yaw < 0:
                    print("Deviating: North")
                    new_location = carla.Location(x = location.x, y = location.y-5, z = location.z)
                    new_rotation = carla.Rotation(pitch=rotation.pitch, yaw=rotation.yaw-5, roll=rotation.roll)
                    ego_vehicle.set_transform(carla.Transform(location, new_rotation))

                if location.x > 0 and location.x < 25 and location.y < 0 and location.y > -25 and rotation.yaw < -100 and rotation.yaw > 200:
                    print("Deviating: West")
                    new_location = carla.Location(x = location.x, y = location.y-5, z = location.z)
                    new_rotation = carla.Rotation(pitch=rotation.pitch, yaw=rotation.yaw-5, roll=rotation.roll)
                    ego_vehicle.set_transform(carla.Transform(location, new_rotation))
                           



            
            #world.tick()
            time.sleep(0.1)
            aaa = world.wait_for_tick()

    except OSError as err:
        print("OS error: {0}".format(err))

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
    except OSError as err:
        print("OS error: {0}".format(err))
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with ego vehicle.')
