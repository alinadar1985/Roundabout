
from utils.data_storage import DataStorageWriter
from utils.carla_utils import *

import random
import time
import cv2

from tqdm import tqdm

sys.path.append("../simulator/carla/PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg")
import carla


# vehicles list
VEHICLES_LIST = ['vehicle.audi.a2',
                 'vehicle.audi.tt',
                 'vehicle.carlamotors.carlacola',
                 'vehicle.citroen.c3',
                 'vehicle.dodge_charger.police',
                 'vehicle.jeep.wrangler_rubicon',
                 #'vehicle.nissan.patrol',
                 'vehicle.ford.mustang',
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


# Definition of Sensors (Position, Settings)

camera1_parameters = {'x': 0.15, 'y': -0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': -45,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera1', 'sensor_type': 'camera'}

camera2_parameters = {'x': 0.15, 'y': 0.00, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 0,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera2', 'sensor_type': 'camera'}

camera3_parameters = {'x': 0.15, 'y': 0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 45,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera3', 'sensor_type': 'camera'}

camera4_parameters = {'x': -0.2, 'y': 0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 90,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera4', 'sensor_type': 'camera'}

camera5_parameters = {'x': -0.6, 'y': 0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 135,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera5', 'sensor_type': 'camera'}

camera6_parameters = {'x': -0.6, 'y': 0.00, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 180,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera6', 'sensor_type': 'camera'}

camera7_parameters = {'x': -0.6, 'y': -0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 225,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera7', 'sensor_type': 'camera'}

camera8_parameters = {'x': -0.2, 'y': -0.55, 'z': 1.65, 'roll': 0, 'pitch': -10, 'yaw': 270,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera8', 'sensor_type': 'camera'}

# birds-eye view
camera10_parameters = {'x': 0, 'y': 0, 'z': 100, 'roll': 0, 'pitch': -90, 'yaw': 90,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera10', 'sensor_type': 'camera'}

# 3rd person view
camera9_parameters = {'x': -7, 'y': 0, 'z': 4, 'roll': 0, 'pitch': -25, 'yaw': 0,
                      'width': 1024, 'height': 768, 'fov': 90,
                      'sensor_label': 'camera9', 'sensor_type': 'camera'}

lidar1_parameters = {'x': 0, 'y': 0, 'z': 2.0, 'roll': 0, 'pitch': 0, 'yaw': 0,
                     'channels': 32, 'range': 100.0*200, 'lower_fov': -30, 'upper_fov': 15, 'points_per_second': 56000*10, 'rotation_frequency': 30,
                     'sensor_label': 'lidar', 'sensor_type': 'lidar'}

gnss_parameters = {'x': 1.5, 'y': 0, 'z': 2.4, 'roll': 0, 'pitch': 0, 'yaw': 0,
                   'sensor_label': 'gnss', 'sensor_type': 'gnss'}

# List with all sensors and its configurations to be used in the simulation
SENSOR_LIST = [camera1_parameters, camera2_parameters, camera3_parameters, camera4_parameters, camera5_parameters, camera6_parameters, camera7_parameters, camera8_parameters,
               gnss_parameters,
               camera9_parameters,
               camera10_parameters,
               lidar1_parameters
               ]


def run_use_case_train(num_vehicles: int, skip_frames: int, output_file_path: str, sensor_list: list, sim_params: dict,
                 save_rgb_as_jpeg: bool = True) -> None:
    """
        run the simulation for training dataset
            - ego vehicle moves
            
        parameters
        ==========

        output_file_path: str

        sensor_list: list with all sensors to be used

        sim_params: dict
            - fps: acquisition frame rate
            - ego_velocity: float m/s 
            - opponents_velocities: list [op1, op2, op3, op4, op5] float m/s must, must be absolute values
    """
    print("> Connecting to localhost:2000")
    client = carla.Client('localhost', 2000)
    client.set_timeout(4.0)

    world = client.get_world()

    spawn_points = world.get_map().get_spawn_points()
    start_pose = random.choice(world.get_map().get_spawn_points())
    spawn_points.remove(start_pose)
    waypoint = world.get_map().get_waypoint(start_pose.location)

    print("> Configuring weather conditions")
    weather = world.get_weather()
    weather.cloudyness = 0.5
    weather.precipitation = 0
    weather.precipitation_deposits = 0
    weather.wind_intensity = 1.0
    weather.sun_azimuth_angle = 30
    weather.sun_altitude_angle = 100
    world.set_weather(weather)

    blueprint_library = world.get_blueprint_library()
    actor_list = list()
    vehicles_list = list()

    print("> Creating Vehicles")
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    number_of_spawn_points = len(spawn_points)
    
    NUMBER_OF_VEHICLES = num_vehicles

    batch = []

    bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    if bp.has_attribute('driver_id'):
        driver_id = random.choice(bp.get_attribute('driver_id').recommended_values)
        bp.set_attribute('driver_id', driver_id)
    bp.set_attribute('role_name', 'autopilot')
    transform = random.choice(world.get_map().get_spawn_points())
    actor = SpawnActor(bp, transform).then(SetAutopilot(FutureActor, True))
    batch.append(actor)

    for n, transform in enumerate(spawn_points):
        if n >= NUMBER_OF_VEHICLES:
            break
        bp = random.choice(blueprint_library.filter(random.choice(VEHICLES_LIST)))
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        if bp.has_attribute('driver_id'):
            driver_id = random.choice(bp.get_attribute('driver_id').recommended_values)
            bp.set_attribute('driver_id', driver_id)
        bp.set_attribute('role_name', 'autopilot')
        actor = SpawnActor(bp, transform).then(SetAutopilot(FutureActor, True))
        batch.append(actor)
    for response in client.apply_batch_sync(batch):
        if response.error:
            print(">>ERROR: ", response.error)
        else:
            vehicles_list.append(response.actor_id)

    #actor_list += opponent_actors

    try:
        time.sleep(2)
        # get all vehicles
        vehicles = world.get_actors().filter('vehicle.*')
        #vehicles = [v for v in vehicles if v != ego_vehicle]
        vehicles = [v for v in vehicles]
        print('> Vehicles number', len(vehicles))
        time.sleep(2)
        ego_vehicle = vehicles[0]

        print(">>> Creating and attaching sensors to EGO")

        sensor_actors, sensor_labels = sensor_factory(world, ego_vehicle, sensor_list)
        actor_list += sensor_actors

        #vehicles = [ego_vehicle] + [v for v in vehicles]
        vehicles_bb = [v for v in vehicles]
        
        print('> Creating data storage file', output_file_path)
        ds = DataStorageWriter(output_file_path, sensor_labels + ['bounding_box', 'vehicle_position', 'sensor_transform',
                                                                  'rect_camera1', 'rect_camera2', 'rect_camera3','rect_camera4',
                                                                  'rect_camera5', 'rect_camera6', 'rect_camera7','rect_camera8'])

        ds.write_matrix('bounding_box', 'extent', np.array([ClientSideBoundingBoxes._create_bb_points(vehicle) for vehicle in vehicles_bb]))
        ds.write_matrix('bounding_box', 'location',
                        np.array([[vehicle.bounding_box.location.x,
                                   vehicle.bounding_box.location.y,
                                   vehicle.bounding_box.location.z] for vehicle in vehicles_bb],dtype=np.float32))

        for idx, params in enumerate(sensor_list):
            ds.write_matrix('sensor_transform',
                            sensor_labels[idx],
                            np.array([params['x'], params['y'], params['z'], params['roll'], params['pitch'], params['yaw']], dtype=np.float32))
        
        print('> Starting simulation')

        with CarlaSyncMode(world, sensor_actors, fps=sim_params['fps']) as sync_mode:
        
            should_quit = False
            for idx in tqdm(range(int(sim_params['fps'])*600)):  # max 1min
            # for idx in range(int(sim_params['fps'])*60):  # max 1min
                
                # for _ in range(skip_frames):
                #     data = sync_mode.tick(timeout=1.0)
                if should_quit:
                    break

                data = sync_mode.tick(timeout=4.0)

                snapshot = data[0]
                frame = snapshot.frame
                ts = int(snapshot.timestamp.elapsed_seconds * 1e6)

                # store data
                sensors_data = data[1:]
                for idx, sensor_label in enumerate(sensor_labels):
                    sensor_data = sensors_data[idx]

                    if sensor_label.startswith('rgb'):
                        if save_rgb_as_jpeg:
                            ds.write_rgb_and_compact(sensor_label, ts, compute_data_buffer(sensor_data))
                        else:
                            ds.write_image(sensor_label, ts, compute_data_buffer(sensor_data))

                        if "camera9" in sensor_label:
                            bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(vehicles, sensor_actors[idx])
                            np_image = compute_data_buffer(sensor_data)
                            # print("np_image", np_image.shape)
                            np_image2 = ClientSideBoundingBoxes.draw_bounding_boxes(np_image, bounding_boxes)

                            cv2.imshow("img", np_image2)
                            if cv2.waitKey(1) == ord('q'):
                                should_quit = True

                    if sensor_label.startswith('depth'):
                        ds.write_matrix(sensor_label, ts, compute_depth_from_buffer(sensor_data))

                    if sensor_label.startswith('semseg'):
                        sensor_data.convert(carla.ColorConverter.CityScapesPalette)
                        ds.write_image(sensor_label, ts, compute_data_buffer(sensor_data))

                    if sensor_label.startswith('lidar'):
                        ds.write_lidar(sensor_label, ts, sensor_data)

                    if sensor_label.startswith('gnss'):
                        ds.write_gnss(sensor_label, ts, sensor_data)
                
                # write vehicle position
                vehicle_pos = np.zeros((len(vehicles_bb), 6))
                for idx, vehicle in enumerate(vehicles_bb):
                    transform = vehicle.get_transform()
                    vehicle_pos[idx, 0] = transform.location.x
                    vehicle_pos[idx, 1] = transform.location.y
                    vehicle_pos[idx, 2] = transform.location.z
                    vehicle_pos[idx, 3] = transform.rotation.roll
                    vehicle_pos[idx, 4] = transform.rotation.pitch
                    vehicle_pos[idx, 5] = transform.rotation.yaw
                ds.write_matrix('vehicle_position', ts, vehicle_pos)

    finally:
        
        try:
            ds.close()
        except:
            pass

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        print("> Cleaning Simulation")        
        for actor in actor_list:
            actor.destroy()
        cv2.destroyAllWindows()


if __name__ == "__main__":

    for idx_train in range(15):
        output_file_path = "../dataset/ext/train%02d.h5" % (idx_train+29)
    
        save_rgb_as_jpeg = True
        num_vehicles = 30
        skip_frames = 10

        run_use_case_train(num_vehicles, skip_frames,
                           output_file_path, SENSOR_LIST,
                           {"fps": 1}, save_rgb_as_jpeg)

