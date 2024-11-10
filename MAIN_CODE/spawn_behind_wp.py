import carla
import math
import random

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

def get_spawn_behind_wp(world, waypoint, distance=10.0, focus=0):
   
    spawn_points = world.get_map().get_spawn_points()
    
    # Trova il punto di spawn dietro al waypoint scelto, che sia distante almeno quanto indicato nella tolleranza
    closest_spawn = find_sp_behind_wp(world, waypoint, spawn_points, distance)
    
    # Disegna una "X" rossa sul waypoint e una "S" verde sul punto di spawn trovato
    if waypoint:
        world.debug.draw_string(
            waypoint.transform.location,
            "X",
            draw_shadow=False,
            color=carla.Color(r=255, g=0, b=0),
            life_time=30.0,
            persistent_lines=False
        )
    
    if closest_spawn:
        if focus == 1:
            camera_location = carla.Location(closest_spawn.location.x, closest_spawn.location.y, z=50) #altezza 60
            camera_rotation = carla.Rotation(pitch=-90)  # Orientamento verso il basso
            # Sposta la telecamera sul punto di spawn trovato
            spectator = world.get_spectator()
            spectator.set_transform(carla.Transform(camera_location, camera_rotation))
        
        # Disegna una "S" verde sul punto di spawn
        world.debug.draw_string(
            closest_spawn.location,
            "S",
            draw_shadow=False,
            color=carla.Color(r=0, g=255, b=0),
            life_time=30.0,
            persistent_lines=False
        )

        print("Trovato spawn")
    else:
        print("Nessun punto di spawn trovato")
        
    return closest_spawn
