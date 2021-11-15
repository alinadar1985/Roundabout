import glob
import os
import sys
import time
import json
import csv 
import traci
import time
import numpy
import random
import math
import weakref
'''
#from geopy.distance import geodesic
#import paho.mqtt.client as mqtt
from cpm_send import sendCPM
'''
from time import gmtime, strftime
import carla
import argparse
import logging
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass





IM_WIDTH = 320
IM_HEIGHT = 240
actor_list =[]
SLICE_NUMBER = 30
LANE_NUMBER = 2
# Initialize global variables #
data = {}                     #
ego_speed = ego_angle = None  #
ID = lat = lon = None         #
objects = []                  #         
vehicles_front = []           #
vehicles_back = []            #
vehicles_collided = []        #
###############################

polygones = []
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


class Triangle:
    def __init__(self, center,back, front):
        self.center = center
        self.back = back
        self.front = front
        self.total_distance = round(math.sqrt(((self.back.x - self.front.x) ** 2) + ((self.back.y - self.front.y) ** 2)),2)
    def is_inside(self, p):
        l1 = (p.x - self.center.x) * (self.back.y - self.center.y) - (self.back.x - self.center.x) * (p.y - self.center.y)
        l2 = (p.x - self.front.x) * (self.center.y - self.front.y) - (self.center.x - self.front.x) * (p.y - self.front.y)
        l3 = (p.x - self.back.x) * (self.front.y - self.back.y) - (self.front.x - self.back.x) * (p.y - self.back.y);
        is_inside = (l1 > 0 and l2 > 0 and l3 > 0) or (l1 < 0 and l2 < 0 and l3 < 0)
        return is_inside

class Vehicle(carla.Actor):
    role: str
    circle_index:int
    slice_index:int
    location:carla.Location
    dist_slices:int
    def __init__(self, role, circle_index, slice_index,location, dist_slices = 1000):
        self.role = role
        self.circle_index = circle_index
        self.slice_index = slice_index
        self.location = location
        self.dist_slices = dist_slices


def getDistance(p1,p2):
    return round(math.sqrt(((p1.x - p2.x) ** 2) + ((p1.y - p2.y) ** 2)), 2)

def spwan_carla(world, map,bp):
    spawn_points = map.get_spawn_points()
    random.shuffle(spawn_points) 
    ego_transform = spawn_points[0]
    ego_transform = carla.Transform(carla.Location(x=42.11999893, y=-2.039999962, z=3.000000),carla.Rotation(pitch=0.000000, yaw=180, roll=0.000000))
    can_spawn = False
    # Now ego can be spawned
    return world.spawn_actor(bp,ego_transform)

def draw_perpendicular(point1,point2, debug):
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y
    alp1 = 45
    alp2 = 90
    u = x2 - x1
    v = y2 - y1
    a3 = math.sqrt(u ** 2 + v ** 2)
    alp3 = 45
    a2 = a3 * math.sin(alp2 * math.pi / 180) / math.sin(alp3 * math.pi / 180);
    RHS1 = x1 * u + y1 * v + a2 * a3 * math.cos(alp1 * math.pi / 180);
    RHS2 = y2 * u - x2 * v - a2 * a3 * math.sin(alp1 * math.pi / 180);
    x3 = (1 / a3 ** 2) * (u * RHS1 - v * RHS2);
    y3 = (1 / a3 ** 2) * (v * RHS1 + u * RHS2);
    arrow_location = carla.Location(x=x3, y=y3, z=0.5)
    debug.draw_line(point2, arrow_location, color=carla.Color(255,255,0), life_time=0.01)
    return arrow_location

def vehicle_in_roundabout(location_center,location_vehicle, raduis):
    dx = abs(location_vehicle.x - location_center.x)
    dy = abs(location_vehicle.y - location_center.y)
    if (dx**2 + dy**2) > raduis**2:
        return False
    return True

def find_angle(pt_base,pt_moved,center):
    angle = math.atan2((pt_moved.y - center.y), (pt_moved.x - center.x)) - math.atan2((pt_base.y - center.y), (pt_base.x - center.x))
    return math.degrees(angle)

def draw_slices(debug):
    lane_number = 2
    circle_count = lane_number + 1
    lane_width = 3.55
    radius_small = 21.5
    x_center = -0.36
    y_center = -0.1
    slice_count =30
    angle_slice = 360/slice_count

    circles = []
    for c in range(circle_count):
        circle = []
        prev_p = None
        countd = 1
        radius = radius_small + (c * lane_width)
        for i in range(slice_count): # slice_count = 30
            angle = angle_slice * countd * (math.pi / 180)
            y = radius * math.cos(angle)
            x = radius * math.sin(angle)
            p = carla.Location(x + x_center, y + y_center)
            circle.append(p)
            if prev_p is not None:
                debug.draw_line(prev_p, p, color=carla.Color(255, 0, 0), life_time=0)
            if i == slice_count-1:
                debug.draw_line(p, circle[0], color=carla.Color(255, 0, 0), life_time=0)
            prev_p = p
            if len(circles) > 0:
                debug.draw_line(circles[c-1][i], p, color=carla.Color(255, 0, 0), life_time=0)
            countd += 1
        circles.append(circle)

    polygones = []
    for c in range(circle_count-1):
        circle1 = circles[c]
        circle2 = circles[c+1]
        polygons_cirlce = []
        for i in range(slice_count-1): # slice_count = 30
            p1 = circle1[i]
            p2 = circle2[i]
            p3 = circle2[i+1]
            p4 = circle1[i+1]
            polygon = Polygon([(p1.x, p1.y),(p2.x, p2.y),(p3.x, p3.y),(p4.x, p4.y)])
            polygons_cirlce.append(polygon)
            if i == slice_count-2:
                p1 = circle1[i+1]
                p2 = circle2[i+1]
                p3 = circle2[0]
                p4 = circle1[0]
                polygon = Polygon([(p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y), (p4.x, p4.y)])
                polygons_cirlce.append(polygon)
        polygones.append(polygons_cirlce)
    return polygones

def draw_polygon(polygon,debug):
    points=[]
    for x, y in polygon.exterior.coords:
       p = carla.Location(x,y)
       points.append(p)
    #print(str(len(points)) + ": ------> " + str(points))
    for i in range(len(points)-1):
        debug.draw_line(points[i], points[i+1], color=carla.Color(0, 0, 255), life_time=0.02)
        debug.draw_line(points[i], points[i+1], color=carla.Color(0, 0, 255), life_time=0.02)

def get_circle_and_slice(point):
    for j in range(30):
        if polygones[0][j].contains(point):
            circle_index = 0
            slice_index = j
            return circle_index, slice_index
        if polygones[1][j].contains(point):
            circle_index = 1
            slice_index = j
            return circle_index, slice_index
    return None, None

def main():
    #traci.init(8813)
    #traci.setOrder(2)
    global polygones
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--host',metavar='H',default='127.0.0.1',help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port',metavar='P',default=2000,type=int,help='TCP port to listen to (default: 2000)')
    args = argparser.parse_args()
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    client = carla.Client(args.host, args.port)
    client.set_timeout(1.0)

    try:
        world = client.get_world()
        blueprints = world.get_blueprint_library()
        ego_vehicle= None

        ego_bp = blueprints.find('vehicle.tesla.model3')
        ego_bp.set_attribute('color','255,0,0')
        ego_bp.set_attribute('role_name','ego')

        carla_map =  world.get_map()
        #map_waypoints = carla_map.generate_waypoints(2.0)
        ego_vehicle = spwan_carla(world, carla_map,ego_bp)
        print('\n Ego is spawned and autopilotted')
        ego_vehicle.set_autopilot(True)
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x=0, y=0, z=90),carla.Rotation(pitch=-90)))
        tm = client.get_trafficmanager(8000) 
        #tm.set_synchronous_mode(True)
        #tm.set_random_device_seed(True)

        debug = world.debug
        lane_number = 2
        lane_width = 3.5
        radius_small = 21.5
        radius_big = radius_small + (lane_width * lane_number)

        current_triangle = None
        center_location = carla.Location(x=-0.36, y=-0.1, z=0.5)
        exit_top_location = carla.Location(x=-3.803, y=31.254, z=0.5)
        exit_right_location = carla.Location(x=30.593, y=3.367, z=0.5)
        exit_bottom_location = carla.Location(x=2.376, y=-32.053, z=0.5)
        exit_left_location = carla.Location(x=-32.134, y=-3.071, z=0.5)
        trg_top_right = Triangle(center_location, exit_top_location, exit_right_location)
        trg_right_bottom = Triangle(center_location, exit_right_location, exit_bottom_location)
        trg_bottom_left = Triangle(center_location, exit_bottom_location, exit_left_location)
        trg_left_top = Triangle(center_location, exit_left_location, exit_top_location)
        triangles = [trg_top_right,trg_right_bottom,trg_bottom_left,trg_left_top]
        polygones = draw_slices(debug)

        while True:
            ego_location = ego_vehicle.get_transform().location
            ego_direction = ego_vehicle.get_transform().get_forward_vector()
            box = ego_vehicle.bounding_box
            bumper_location = ego_location + (box.extent.x * ego_direction)
            ego_direction_location = bumper_location + (15 * ego_direction)
            lateral_position = round((getDistance(ego_location,center_location)-radius_small),2)
            debug.draw_string(carla.Location(-25,25),"Laternal Position: " + str(lateral_position)+"/ 7", False, carla.Color(0, 0, 0), 0.005)

            for trg in triangles:
                if trg.is_inside(bumper_location):
                    current_triangle = trg

            if current_triangle is not None:
                exit_distance = getDistance(bumper_location, current_triangle.front)
                total_distance = current_triangle.total_distance
                debug.draw_line(bumper_location, current_triangle.front, color=carla.Color(0, 255, 255), life_time=0.01)
                debug.draw_string(current_triangle.front, "Exit_distance: " + str(exit_distance)+"/"+str(total_distance), False, carla.Color(0, 0, 0),0.01)

            debug.draw_line(bumper_location, center_location, color=carla.Color(255,255,0),life_time = 0.01)
            tang_location = draw_perpendicular(center_location, bumper_location, debug)
            debug.draw_arrow(bumper_location,ego_direction_location ,0.5,color=carla.Color(255,0,0), life_time=0.01)
            heading_angle = -(find_angle(tang_location, ego_direction_location, bumper_location))
            world.debug.draw_string(carla.Location(-28,25),"Heading angle: " +  str(round(heading_angle,2)), False, carla.Color(0, 0, 0, 0), 0.01)
            world.debug.draw_string(ego_direction_location, "D", False, carla.Color(0, 0, 0, 0), 0.01)
            world.debug.draw_string(tang_location, "T", False, carla.Color(0, 0, 0, 0), 0.01)

            tm.force_lane_change(ego_vehicle,False)

            actors = world.get_actors().filter('vehicle.*')
            count = 0
            my_egos = [x for x in actors if x.attributes['role_name'] == 'ego' ]
            my_others = [x for x in actors if x.attributes['role_name'] != 'ego' ]
            vehicles = []
            for actor in actors:
                actor_location  = actor.get_transform().location
                if vehicle_in_roundabout(center_location, actor_location, radius_big):
                    point = Point(actor_location.x, actor_location.y)
                    my_role = 'ego' if actor.attributes['role_name'] == 'ego' else 'other'
                    circle, slice = get_circle_and_slice(point)
                    veh = Vehicle(my_role,circle,slice, actor_location)
                    if circle is None or slice is None: continue
                    draw_polygon(polygones[circle][slice],debug)
                    vehicles.append(veh)
                    count+=1
            debug.draw_string(carla.Location(00, 0), str(count) + " cars inside roundabout", False, carla.Color(0, 0, 0, 0), 0.01)

            for lane in range(lane_number):
                vehs_circle = [x for x in vehicles if x.circle_index == lane ]
                for v1 in vehs_circle:
                    for v2 in [x for x in vehs_circle if x.location != v1.location]:
                        dist = 0
                        if v1.slice_index < v2.slice_index:
                            dist = v2.slice_index - v1.slice_index
                        if v1.slice_index > v2.slice_index:
                            dist = SLICE_NUMBER - (v1.slice_index - v2.slice_index)
                        v1.dist_slices = dist if dist < v1.dist_slices else v1.dist_slices
                    print("Vehicle: " + v1.role + "(" + str(v1.circle_index) + ") @ " + str(v1.slice_index) + "--> " + str(v1.dist_slices))
                    print("---------------------------------------------------------------------------")




            world_snapshot = world.wait_for_tick()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()




if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n Done with ego vehicle.')


















'''
        counter = 0
        #ego_vehicle.set_simulate_physics(False)

        with open('locations.csv', 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)



        new_wp = waypoint.get_left_lane()

         with open('locations.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        for s in spawn_points:
            writer.writerow([s.location.x,s.location.y,s.location.z])
    with open('rotations.csv', 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        for s in spawn_points:
            writer.writerow([s.rotation.pitch,s.rotation.yaw,s.rotation.roll])
'''

'''
    print("Start finding safe point...")
    while not can_spawn:
        conflicted = 0
        lenght = len(traci.vehicle.getIDList())
        print("Number of vehicles = " + str(lenght))
        for vehicle_id in traci.vehicle.getIDList():
            x, y = traci.vehicle.getPosition(vehicle_id)
            sumo_speed = traci.vehicle.getSpeed(vehicle_id)
            sumo_angle = traci.vehicle.getAngle(vehicle_id)
            distance_between = math.sqrt((location_spawn.x-x)**2 +(location_spawn.y -y)**2)
            time_between = distance_between/sumo_speed
            if time_between < spwan_threshold:
                conflicted = conflicted + 1
        
        if conflicted == 0:
            can_spawn = True
            break
        print("Can not spwan Ego vehicle...")
        time.sleep(1)
'''

'''
carla.Timestamp
---------------
elapsed_seconds (float)
Simulated seconds elapsed since the beginning of the current episode.
delta_seconds (float)
Simulated seconds elapsed since the previous frame.
platform_timestamp (float)
Time register of the frame at which this measurement was taken given by the OS in seconds.
'''
