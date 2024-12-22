import carla
import random
import math
import threading
import lib_spawn as spb
import lib_map_draw as dr
import json
from importlib import reload
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.behavior_agent import BehaviorAgent
from collections import defaultdict

reload(spb) #per refreshare modifiche import

# Tolleranza per considerare i waypoints sullo stesso asse X o Y
tolerance = 2
#contatore di tutte le coppe di wp
pair_counter = 0
pair_indexer = defaultdict(int)

#cancella tutti gli attori del simulatore
def reset_sim(world):
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for sen in world.get_actors().filter('*sensor*'):
        sen.destroy()

#lista di adiacenza di coppie di waypoints avendi il secondo waypoint in comune
#significa che hanno punti di partenza differenti ma si intersecano nello stesso waypoint 
def group_pairs_by_second_waypoint(waypoint_pairs):
    
    # Dizionario (mappa) per raggruppare le coppie in base alle coordinate (x, y) del secondo waypoint di ciascuna coppia
    coord_groups = defaultdict(list)
    
    # Raggruppa le coppie in base alle coordinate x e y del secondo waypoint
    for wp1, wp2 in waypoint_pairs:
        #la chiave è rappresentata dalle coordinate del secondo wp
        coord_wp2 = (round(wp2.transform.location.x, 3), round(wp2.transform.location.y, 3))
        coord_groups[coord_wp2].append((wp1, wp2))
    
    # Stampa i gruppi di coppie per ciascun secondo waypoint
    '''for wp2_coords, pairs in coord_groups.items():
        if len(pairs) > 1:
            print(f"Le coordinate {wp2_coords} accomunano le seguenti coppie:")
            for pair in pairs:
                print(p_wp(pair[0]) + ", " + p_wp(pair[1]))
            print() '''
        
    return coord_groups  

#print waypoint coordinates aproximated to third digit
def p_wp(w):
  return f"({round(w.transform.location.x,3)}) - ({round(w.transform.location.y,3)})"

#reads json rows from files
def read_json_scenarios(file_name):
    json_list = []
    try:
        with open(file_name, "r", encoding="utf-8") as file:
            for i, r in enumerate(file, start=1):
                if i == 2:
                    break
                r = r.strip()
                if r: #check if trimmed row is not empty
                    try:
                        json_obj = json.loads(r)  #read row as json struct
                        json_list.append(json_obj)
                    except json.JSONDecodeError as e:
                        print(f"Error in row {i + 1}: invalid JSON format ({e})")
        return json_list
    except FileNotFoundError:
        print(f"File '{file_name}' missing!!")
    except Exception as e:
        print(f"Error during parsing file '{file_name}': {e}")
    return []