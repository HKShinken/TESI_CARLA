# https://carla.readthedocs.io/en/0.9.12/adv_agents/#:~:text=set_destination(end_location%2C%20start_location%3DNone,use%20the%20current%20agent%20location
#BEHAVIOUR AGENT CON SETTAGGIO DESTINAZIONE

#all imports
import carla #the sim library itself
import cv2 #to work with images from cameras
import time # towork with images from cameras
import numpy as np #in this example to change image representation - re-shaping
import math
import random


min_dist = 1000

def reset_min_dist():
    global min_dist  # Usa la parola chiave 'global' per modificare la variabile globale
    min_dist = 1000

def attach_sensors(world, vehicle, max_dist_check = 5, tol = 2):

    collision_check = False
    ###############################################      INIZIALIZZAZIONE SENSORI      #####################################################
    # SENSORE TELECAMERA PER FINESTRA PYGAME
    bp_lib = world.get_blueprint_library()
    camera_bp = bp_lib.find('sensor.camera.rgb') 
    camera_init_trans = carla.Transform(carla.Location(z=2))
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    
    # sensore di posizione
    gnss_bp = bp_lib.find('sensor.other.gnss')
    gnss_sensor = world.spawn_actor(gnss_bp, carla.Transform(), attach_to=vehicle)
    
    # sensore di misurazione movimento inerziale 
    imu_bp = bp_lib.find('sensor.other.imu')
    imu_sensor = world.spawn_actor(imu_bp, carla.Transform(), attach_to=vehicle)
    
    # sensore rilevazione collisione
    collision_bp = bp_lib.find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
    
    # sensore invasione di corsia
    lane_inv_bp = bp_lib.find('sensor.other.lane_invasion')
    lane_inv_sensor = world.spawn_actor(lane_inv_bp, carla.Transform(), attach_to=vehicle)
    
    # attributi sensore ostacoli
    obstacle_bp = bp_lib.find('sensor.other.obstacle')
    obstacle_bp.set_attribute('hit_radius','0.5') #detection radius, deault 0.5
    obstacle_bp.set_attribute('distance',str(max_dist_check)) #detection range, default 50
    obstacle_bp.set_attribute('only_dynamics', 'True')  # only vehicle or pedestrians
    obstacle_sensor = world.spawn_actor(obstacle_bp, carla.Transform(), attach_to=vehicle)
    
    #################################### FUNZIONI DI CALLBACK SENSORI #########################################
    def rgb_callback(image, data_dict):
        data_dict['rgb_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    
    def gnss_callback(data, data_dict):
        data_dict['gnss'] = [data.latitude, data.longitude]
    
    def imu_callback(data, data_dict):
        data_dict['imu'] = {
            'gyro': data.gyroscope,
            'accel': data.accelerometer,
            'compass': data.compass
        }
        
    def lane_inv_callback(event, data_dict):
        data_dict['lane_invasion'] == True
        
    def collision_callback(event, data_dict):
        data_dict['collision'] = True
        
    #obstacle sensors, dtecting ditance and involved actor  
    def obstacle_callback(event, data_dict, camera, k_mat):
        global min_dist
        if event.distance < min_dist:
            min_dist = event.distance
        
        world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
        image_point = get_image_point(event.other_actor.get_transform().location, k_mat, world_2_camera)
        if  0 < image_point[0] < image_w and 0 < image_point[1] < image_h:
            cv2.circle(data_dict['rgb_image'], tuple(image_point), 10, (0,0,255), 3)
    
    # strutture dati ausiliare per poriezione coordinate su schermo
    def build_projection_matrix(w, h, fov):
        focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
        K = np.identity(3)
        K[0, 0] = K[1, 1] = focal
        K[0, 2] = w / 2.0
        K[1, 2] = h / 2.0
        return K
    
    def get_image_point(loc, K, w2c):
            # Calculate 2D projection of 3D coordinate
    
            # Format the input coordinate (loc is a carla.Position object)
            point = np.array([loc.x, loc.y, loc.z, 1])
            # transform to camera coordinates
            point_camera = np.dot(w2c, point)
    
            # conversione coordinate UE4's verso quelle standard
            # (x, y ,z) -> (y, -z, x)
            # rimuovendo la quarta componente
            point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
    
            # now project 3D->2D using the camera matrix
            point_img = np.dot(K, point_camera)
            # normalize
            point_img[0] /= point_img[2]
            point_img[1] /= point_img[2]
    
            return tuple(map(int, point_img[0:2]))
        
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    
    # Get the attributes from the camera
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()
    
    # Calculate the camera projection matrix to project from 3D -> 2D
    K = build_projection_matrix(image_w, image_h, fov)
    
    #################################################  AVVIO DEI SENSORI ############################################
    collision_counter = 20
    lane_invasion_counter = 20
    sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4)),
                   'collision': False,
                   'lane_invasion': False,
                   'gnss': [0,0],
                   'obstacle': [],
                   'imu': {
                        'gyro': carla.Vector3D(),
                        'accel': carla.Vector3D(),
                        'compass': 0
                    }}
    
    # OpenCV window with initial data
    cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE) 
    cv2.imshow('RGB Camera', sensor_data['rgb_image'])
    cv2.waitKey(1)
    
    # Start sensors recording data
    camera.listen(lambda image: rgb_callback(image, sensor_data))
    collision_sensor.listen(lambda event: collision_callback(event, sensor_data))
    gnss_sensor.listen(lambda event: gnss_callback(event, sensor_data))
    imu_sensor.listen(lambda event: imu_callback(event, sensor_data))
    lane_inv_sensor.listen(lambda event: lane_inv_callback(event, sensor_data))
    obstacle_sensor.listen(lambda event: obstacle_callback(event, sensor_data, camera, K))
    
    # Parameters for text on screen
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,50)
    fontScale              = 0.5
    fontColor              = (255,255,255)
    thickness              = 2
    lineType               = 2
    
    # Draw compass (in radiants) with line towards cardinal points
    def draw_compass(img, theta):
        compass_center = (700, 100)
        compass_size = 50
        
        cardinal_directions = [
            ('N', [0,-1]),
            ('E', [1,0]),
            ('S', [0,1]),
            ('W', [-1,0])
        ]
        for car_dir in cardinal_directions:
            cv2.putText(sensor_data['rgb_image'], car_dir[0], 
            (int(compass_center[0] + 1.2 * compass_size * car_dir[1][0]), int(compass_center[1] + 1.2 * compass_size * car_dir[1][1])), 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)
        
        compass_point = (int(compass_center[0] + compass_size * math.sin(theta)), int(compass_center[1] - compass_size * math.cos(theta)))
        cv2.line(img, compass_center, compass_point, (255, 255, 255), 3)
    	
    	
    ################################### SENSOR MAIN CODE  ################################################
    while True:
        #SE L'AGENT HA CONCLUSO COLORA DI BIANCO IL PERCORSO COMPLETATO
        if  cv2.waitKey(1) == ord('q') or vehicle.is_alive ==  False:# or min_dist < tol :
            if min_dist < tol:
               print(f"Distanza di sicurezza {tol} non rispettata, fine simulazione") 
            if vehicle.is_alive ==  False:
               print("EGO vehicle has reached his destination, end of simulation")
            break
        
        # Latitude sensor GNSS
        cv2.putText(sensor_data['rgb_image'], 'Lat: ' + str(sensor_data['gnss'][0]), 
        (10,30), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
        
        # Longitude sensor GNSS
        cv2.putText(sensor_data['rgb_image'], 'Long: ' + str(sensor_data['gnss'][1]), 
        (10,50), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
        
        #acceleration vector minus gravity (-9.81)
        accel = sensor_data['imu']['accel'] - carla.Vector3D(x=0,y=0,z=9.81)
        
        # acceleration magnitude
        cv2.putText(sensor_data['rgb_image'], 'Accel: ' + str(accel.length()), 
        (10,70), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
        
        # Gyroscope
        cv2.putText(sensor_data['rgb_image'], 'Gyro: ' + str(sensor_data['imu']['gyro'].length()), 
        (10,100), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
        
        # bussola, nord è zero radianti
        cv2.putText(sensor_data['rgb_image'], 'Compass: ' + str(sensor_data['imu']['compass']), 
        (10,120), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)

         # Current detected dist from next obstacle
        cv2.putText(sensor_data['rgb_image'], 'Tolerance dist: ' + str(tol), 
        (10,140), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)

        # Current detected dist from next obstacle
        cv2.putText(sensor_data['rgb_image'], 'Detected Obstacle distance: ' + str(min_dist), 
        (10,160), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)

        # Current detected dist from next obstacle
        cv2.putText(sensor_data['rgb_image'], 'PRESS Q TO EXIT THIS WINDOWS ', 
        (10,180), 
        font, 
        fontScale,
        fontColor,
        thickness,
        lineType)
        
        # Draw compass on screen
        draw_compass(sensor_data['rgb_image'], sensor_data['imu']['compass'])
        
        # print 'COLLISION' when flag sensor is true, lasts 20 frames
        if sensor_data['collision']:
            collision_counter -= 1
            collision_check = True
            if collision_counter <= 1:
                sensor_data['collision'] = False
            cv2.putText(sensor_data['rgb_image'], 'COLLISION', 
            (250, 300), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            2,
            (255,255,255),
            3,
            2)
        else:
            collision_counter = 20
            
        # stampa 'LANE INVASION' uando il flag del sensore è true, dura 20 fotogrammi
        if sensor_data['lane_invasion']:
            lane_invasion_counter -= 1
            if lane_invasion_counter <= 1:
                sensor_data['lane_invasion'] = False
            cv2.putText(sensor_data['rgb_image'], 'LANE INVASION', 
            (190, 350), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            2,
            (255,255,255),
            3,
            2)
        else:
            lane_invasion_counter = 20
         
        # mostra il POV del veicolo in una finestra pygame separata
        cv2.imshow('RGB Camera', sensor_data['rgb_image'])
        
        # termina il loop se l'utente preme q 
        if cv2.waitKey(1) == ord('q'):
            break
     
    camera.stop()
    collision_sensor.stop()
    gnss_sensor.stop()
    imu_sensor.stop()
    lane_inv_sensor.stop()
    obstacle_sensor.stop()
    cv2.destroyAllWindows()
    return (not(vehicle.is_alive), not(min_dist) < tol, min_dist, collision_check)
