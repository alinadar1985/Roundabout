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
import matplotlib.pyplot as plt
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

        carla_map =  world.get_map()
        map_waypoints = carla_map.generate_waypoints(1.0)
        #spawn_points = world.get_map().get_spawn_points()
        #world.unload_map_layer(carla.MapLayer.Buildings)

        #test_location = carla.Location(0,0,0)
        #number_of_spawn_points = len(spawn_points)
        #waypoints = carla_map.generate_waypoints(15)

        x = [p.transform.location.x for p in map_waypoints]
        y = [p.transform.location.y for p in map_waypoints]

        file = open('topology.csv', 'a', newline='')
        writer = csv.writer(file, delimiter = ",")
        writer.writerow(("X","Y"))
        for i in range(len(x)):
            writer.writerow((x[i],y[i]))
        file.close()

       
        #plt.plot(x, y, marker = '.')
        plt.scatter(x, y, s=0.5,marker='.')
        plt.savefig("topology.png")
        plt.show()
    
        #world.tick()
        #time.sleep(0.1)
        world.wait_for_tick()

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
