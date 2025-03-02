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
    min_distance = float('inf')
    
    # Direzione del waypoint come vettore
    waypoint_direction = waypoint.transform.get_forward_vector()
    waypoint_lane_id = waypoint.lane_id  # Identificatore della corsia del waypoint

    #per ogni spawn point della mappa
    for spawn in spawn_points:
        # Calcola la distanza tra il punto di spawn e il waypoint
        distance = math.sqrt(
            (spawn.location.x - waypoint.transform.location.x) ** 2 +
            (spawn.location.y - waypoint.transform.location.y) ** 2
        )
        
        # Verifica se il punto di spawn distante almeno a "tolerance" metri dal wp considerato
        if distance >= tolerance:
            # converte lo spawn point in eame in waypoint di tipo Driving (guida) in modo ne possa recuperare l'id lane
            spawn_waypoint = world.get_map().get_waypoint(spawn.location, project_to_road=True, lane_type=carla.LaneType.Driving)
            
            #stessa corsia del wp in input
            if spawn_waypoint and spawn_waypoint.lane_id == waypoint_lane_id:
                # vettore direzione dal waypoint al punto di spawn
                direction_to_spawn = carla.Vector3D(
                    spawn.location.x - waypoint.transform.location.x,
                    spawn.location.y - waypoint.transform.location.y,
                    0
                )
                
                # Calcola il prodotto scalare per verificare se il punto di spawn è dietro al waypoint
                dot_product = (direction_to_spawn.x * waypoint_direction.x + 
                               direction_to_spawn.y * waypoint_direction.y)
                
                # Se il prodotto scalare è negativo, il punto è dietro al waypoint
                if dot_product < 0 and distance < min_distance:
                    min_distance = distance
                    closest_spawn = spawn
    
    return closest_spawn

#richiama find_sp_behind_wp e stampa a video lo spawn restituito
#opzionalmente è possibile focalizzare lo spettatore sullo spwan
def get_spawn_behind_wp(world, waypoint, distance=10.0, focus=0):
   
    spawn_points = world.get_map().get_spawn_points()
    
    # Trova il punto di spawn dietro al waypoint scelto, che sia distante almeno quanto indicato nella tolleranza
    closest_spawn = find_sp_behind_wp(world, waypoint, spawn_points, distance)
    
    #DEBUG
    '''
    if waypoint:
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
        world.debug.draw_string(
            closest_spawn.location,
            "Spawn",
            draw_shadow=False,
            color=carla.Color(r=0, g=255, b=0),
            life_time=15.0,
            persistent_lines=False
        )

        #print(f"Spawn per {waypoint} di id_lane {waypoint.lane_id} ALLE COORDINATE {closest_spawn}")
    else:
        print(f"Nessun punto di spawn trovato per {waypoint}")
        
    return closest_spawn

#ausiliaria per spawnare veicolo definendo opzionalmente autopilota o il tipo di veicolo
def spawn_vehicle(world, spawn_point, autopilot=False, vehicle_id='NA'):
    
    if vehicle_id == 'NA':
       vehicle_id = 'vehicle.lincoln.mkz_2020'

    bp_lib = world.get_blueprint_library() 
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    if autopilot == True:
       vehicle.set_autopilot(autopilot)
    return vehicle


#funzione ausiliaria che data una lista di coppie di wp spawna il veicolo dietro ogni priomo wp di ogni coppia
def spawn_all_behind_wp_couple(world, pairs, distance, focus):
    vehicle_list = []
    tmp = 0
    for idx, (wp1, wp2) in enumerate(pairs):
        if idx == len(pairs) - 1 and focus == 1:
            tmp = 1
        spawn_point = get_spawn_behind_wp(world, wp1, distance, tmp)
        vehicle_list.append( (spawn_vehicle(world, spawn_point), wp2) ) #, autopilot=False, vehicle_id='NA')
        idx = idx + 1
    return vehicle_list

#funzione ausiliaria che dato un insieme di liste di coppie di wp che si incrociano restituiser una lista di spawn ditro a ciascun primo wp 
def list_all_behind_wp_couple(world, grouped_pairs, distance):
    spawn_list = []
    
    for wp2_coords, pairs in grouped_pairs.items():
        if len(pairs) > 1:
           dr.draw_intersection(world, pairs)
           for (wp1, wp2) in pairs:
              spawn_list.append( get_spawn_behind_wp(world, wp1, distance, 0) ) 

    return spawn_list

#calcola la distanza in metri tra due wp
def wp_distance(wp1, wp2):
    distance = 999999
    if wp1 is not None and wp2 is not None:
        distance =  wp1.transform.location.distance(wp2.transform.location)
    return distance

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


