import carla
import math
import random
from collections import defaultdict
import lib_map_draw as dr
from importlib import reload
reload(dr)

#dato un waypoint restituisce lo spwan dietro ad esso distante almeon 10 metri
#WARNING può capitare che due wp su corise differenti abbiano lo stesso lane id...
def find_sp_behind_wp(world, waypoint, spawn_points, tolerance=10.0):

    closest_spawn = None
    min_distance = 999
    
    # wp direction as vector
    waypoint_direction = waypoint.transform.get_forward_vector()
    waypoint_lane_id = waypoint.lane_id  # id lane of current wp

    for spawn in spawn_points:
        distance = math.sqrt(
            (spawn.location.x - waypoint.transform.location.x) ** 2 +
            (spawn.location.y - waypoint.transform.location.y) ** 2
        )
        
        # check if the distance between wp and sp is lower than tolerance
        if distance <= tolerance:
            #print(tolerance)
            #print(distance)
            # gets the waypoint corresponding to spawn point in order to get lane id
            spawn_waypoint = world.get_map().get_waypoint(spawn.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            
            #check if converted sp has same lane id as input wp
            if spawn_waypoint and spawn_waypoint.lane_id == waypoint_lane_id:
                # vettore direzione dal waypoint al punto di spawn
                direction_to_spawn = carla.Vector3D(
                    spawn.location.x - waypoint.transform.location.x,
                    spawn.location.y - waypoint.transform.location.y,
                    0
                )
                
                # dot product to check if sp is behind wp
                dot_product = (direction_to_spawn.x * waypoint_direction.x + 
                               direction_to_spawn.y * waypoint_direction.y)
                
                # if negative, sp is behind wp
                if dot_product < 0 and distance < min_distance:
                    min_distance = distance
                    closest_spawn = spawn
    return closest_spawn


#richiama find_sp_behind_wp e stampa a video lo spawn restituito
#opzionalmente è possibile focalizzare lo spettatore sullo spwan
def get_spawn_behind_wp(world, waypoint, distance, focus=0):
   
    spawn_points = world.get_map().get_spawn_points()
    
    # Trova il punto di spawn dietro al waypoint scelto, che sia distante almeno quanto indicato nella tolleranza
    closest_spawn = find_sp_behind_wp(world, waypoint, spawn_points, distance)
    
    
    '''if waypoint:
        world.debug.draw_string(waypoint.transform.location,
            str(waypoint.transform.location.x),
            draw_shadow=True,color=carla.Color(r=255, g=0, b=0),life_time=30.0,persistent_lines=False)'''
    
    if closest_spawn:
        if focus == 1:
            camera_location = carla.Location(closest_spawn.location.x, closest_spawn.location.y, z=50) #altezza 60
            camera_rotation = carla.Rotation(pitch=-90)  # Orientamento verso il basso
            # Sposta la telecamera sul punto di spawn trovato
            spectator = world.get_spectator()
            spectator.set_transform(carla.Transform(camera_location, camera_rotation))
        
        # Scrive "Spawn" verde sul punto di spawn
        '''world.debug.draw_string(
            closest_spawn.location,
            "SPAWN",
            draw_shadow=False,
            color=carla.Color(r=0, g=255, b=0),
            life_time=15,
            persistent_lines=False
        )'''

        #print(f"Spawn per {waypoint} di id_lane {waypoint.lane_id} ALLE COORDINATE {closest_spawn}")
    else:
        print(f"Nessun punto di spawn trovato per {waypoint}")
        
    return closest_spawn

#ausiliaria per spawnare veicolo definendo opzionalmente autopilota o il tipo di veicolo
#nb: il parametro spawn_point è una coppia (spawn, wp_destinazione)
def spawn_vehicle(world, spawn_point, vehicle_id='NA'):
    
    if vehicle_id == 'NA':
       vehicle_id = 'vehicle.lincoln.mkz_2020'

    bp_lib = world.get_blueprint_library() 
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point[0])
    print(f"Spawned vehicle at: {spawn_point[0]}")
    return (vehicle, spawn_point[1], spawn_point[0])

#spwna i veicoli nella lista degli spawn passata in input, n.b: la lista è fatta da coppia (spwan, wp_Dest)
def spawn_traffic(world, spawn_list):

    vehicle_list = []
    vehicle_id = 'vehicle.lincoln.mkz_2020'
    bp_lib = world.get_blueprint_library() 

    for s, w in spawn_list:
       vehicle_bp = random.choice(bp_lib.filter('vehicle.*'))
       vehicle = world.try_spawn_actor(vehicle_bp, s)
       if vehicle is not None:
           vehicle_list.append((vehicle, w, s))

    return vehicle_list;


#funzione ausiliaria che data una lista di coppie di wp spawna il veicolo dietro ogni primo wp di ogni coppia
def spawn_all_behind_wp_couple(world, pairs, distance, focus):
    vehicle_list = []
    foc_tmp = 0
    for idx, (wp1, wp2) in enumerate(pairs):
        if idx == len(pairs) - 1 and focus == 1:
            foc_tmp = 1
        spawn_point = get_spawn_behind_wp(world, wp1, distance, foc_tmp)
        vehicle_list.append(spawn_vehicle(world, spawn_point)) #, autopilot=False, vehicle_id='NA')
        idx = idx + 1
    return vehicle_list

#funzione ausiliaria che dato un insieme di liste di coppie di wp che si incrociano restituiser una lista di spawn dietro a ciascun primo wp di ciascuna coppia
def list_all_behind_wp_couple(world, grouped_pairs, distance):
    spawn_list = []
    
    for wp2_coords, pairs in grouped_pairs.items():
        if len(pairs) > 1:
           for (wp1, wp2) in pairs:
              sp = get_spawn_behind_wp(world, wp1, distance, 0)
              if sp is not None:
                 spawn_list.append( (sp, wp2) ) 

    return sorted( spawn_list, key=lambda tuple: (tuple[0].location.x, tuple[0].location.y, tuple[1].transform.location.x, tuple[1].transform.location.y) )

#calcola la distanza in metri tra due wp
def wp_distance(wp1, wp2):
    distance = 999999
    if wp1 is not None and wp2 is not None:
        distance =  wp1.transform.location.distance(wp2.transform.location)
    return abs(distance)

#routine che rileva se un veicolo ha superato un semaforo rosso
def has_crossed_red_light(vehicle, world):
    # Ottieni semaforo antistante il veicolo
    traffic_light = vehicle.get_traffic_light()
    if traffic_light is None:
        return False  # Nessun semaforo associato

    #stampa di debug
    stato = str(traffic_light.get_state())
    world.debug.draw_string(traffic_light.get_location(), 'SEMAFORO ' + stato, draw_shadow=False, color=carla.Color(255, 0, 0), life_time=0.1)
    if traffic_light.get_state() != carla.TrafficLightState.Red:
       return False

    # Ottieni il waypoint del veicolo
    vehicle_location = vehicle.get_location()
    vehicle_waypoint = world.get_map().get_waypoint(vehicle_location)

    # waypoint della striscia di stop del semaforo
    stop_line_waypoints = traffic_light.get_stop_waypoints()
    if not stop_line_waypoints:
        return False  # Nessuna linea di stop trovata

    stop_line_waypoint = stop_line_waypoints[0]

    # Calcola il vettore tra veicolo e linea di stop
    vehicle_to_stop_line = vehicle_location - stop_line_waypoint.transform.location

    # Ottieni la direzione della corsia (vettore normalizzato)
    lane_direction = stop_line_waypoint.transform.get_forward_vector()

    # Proietta il vettore veicolo-linea di stop lungo la direzione della corsia
    dot_product = (vehicle_to_stop_line.x * lane_direction.x +
                   vehicle_to_stop_line.y * lane_direction.y +
                   vehicle_to_stop_line.z * lane_direction.z)

    # Se il risultato è maggiore di zero, il veicolo ha superato la linea di stop
    return dot_product > 0

#recupera un veicolo allo spawn point specificato
def get_vehicle_at_spawn_point(spawn_point, vehicles, r=3):

    # Ottieni le coordinate del punto di spawn arrotondate
    spawn_x = round(spawn_point[0].location.x, r)
    spawn_y = round(spawn_point[0].location.y, r)
    
    if len(vehicles) != 0:
        for vehicle, w_dest, spawn in vehicles:
            # Ottieni la posizione del veicolo
            vehicle_location = vehicle.get_transform().location
            
            # Arrotonda le coordinate del veicolo
            vehicle_x = round(vehicle_location.x, r)
            vehicle_y = round(vehicle_location.y, r)
    
            if vehicle_x == spawn_x and vehicle_y == spawn_y:
                return (vehicle, w_dest, spawn)

    return None

#rimuove dalla lista dei veicoli NPC l'ego vehicle passato in put
def recompute_npc_list(vehicle_list, target_pair):

    target_vehicle, target_waypoint, sp = target_pair
    
    def round_coordinates(location):
        """Restituisce le coordinate arrotondate alla terza cifra decimale."""
        return round(location.x, 3), round(location.y, 3), round(location.z, 3)

    target_vehicle_coords = round_coordinates(target_vehicle.get_transform().location)
    target_waypoint_coords = round_coordinates(target_waypoint.transform.location)

    return [
        (vehicle, waypoint, sp)
        for vehicle, waypoint, sp in vehicle_list
        if (
            round_coordinates(vehicle.get_transform().location) != target_vehicle_coords or
            round_coordinates(waypoint.transform.location) != target_waypoint_coords
        )
    ]

#confronta due coordinate, non usa z perchè uno spwan può essere più altro di un wp
def compare_location(loc1, loc2):

    def round_coordinates(location):
        """Restituisce le coordinate x e y arrotondate alla terza cifra decimale."""
        return round(location.x, 3), round(location.y, 3)

    # Confronto delle coordinate x e y approssimate alla terza cifra decimale
    return round_coordinates(loc1) == round_coordinates(loc2)

#ricava un waypoint a partire da uno spawn point
def get_wp_from_sp(world, sp):
    return  world.get_map().get_waypoint(sp.location, lane_type=carla.LaneType.Driving)
#recupera un waypoint a partire da un veicolo
def get_wp_from_vh(world, vh):
    return  world.get_map().get_waypoint(vh.get_location(), lane_type=carla.LaneType.Driving)
