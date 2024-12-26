import carla
import random
import math
import threading
import lib_spawn as spb
import lib_map_draw as dr
import json
import tkinter as tk
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

#update simulator with selected data from scenario file
def update_sim_window(self):

    #destroy all current actors and sensors
    for actor in self.world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for sen in self.world.get_actors().filter('*sensor*'):
        sen.destroy()
    
    self.crossing_index = self.uploaded_scenarios[self.current_scenario]['junction_id'] - 1
    self.spawn_index = self.uploaded_scenarios[self.current_scenario]['spawn_id'] - 1

    p_ego = self.uploaded_scenarios[self.current_scenario]['ego_agent']
    p_ego = p_ego[0].upper() + p_ego[1:]
    self.spawn_profile.set(p_ego)

    p_trf = self.uploaded_scenarios[self.current_scenario]['traffic_agent']
    p_trf = p_trf[0].upper() + p_trf[1:]
    self.traffic_profile.set(p_trf)
    
    #getting spawns and destinations
    ego_spawn = self.uploaded_scenarios[self.current_scenario]['ego_spawn']
    npc_spawn = self.uploaded_scenarios[self.current_scenario]['npc_spawn']
    ego_dest = self.uploaded_scenarios[self.current_scenario]['ego_dest']
    npc_dest = self.uploaded_scenarios[self.current_scenario]['npc_dest']

    #creating spawn objects
    lead_spawn = carla.Transform(carla.Location(x=npc_spawn['x'], y=npc_spawn['y'], z=npc_spawn['z']), carla.Rotation(pitch=npc_spawn['pitch'], yaw=npc_spawn['yaw'], roll=npc_spawn['roll']))
    
    lag_spawn = carla.Transform(carla.Location(x=ego_spawn['x'], y=ego_spawn['y'], z=ego_spawn['z']), carla.Rotation(pitch=ego_spawn['pitch'], yaw=ego_spawn['yaw'], roll=ego_spawn['roll']))

    #creating waypoint destination
    carla_map = self.world.get_map()
    dest_lag_wp = carla_map.get_waypoint( carla.Location(x=ego_dest['x'], y=ego_dest['y'], z=ego_dest['z']) )
    dest_lead_wp = carla_map.get_waypoint( carla.Location(x=npc_dest['x'], y=npc_dest['y'], z=npc_dest['z']) )

    bp_lib = self.world.get_blueprint_library() 
    vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
    
    #spawning ego and lead
    ego_v = self.world.try_spawn_actor(vehicle_bp, lag_spawn)
    self.ego_vehicle = (ego_v, dest_lag_wp, lag_spawn)
    self.ego_spawn_location = (lag_spawn, dest_lag_wp)
    #print(ego_v)
    
    lead_v = self.world.try_spawn_actor(vehicle_bp, lead_spawn)
    self.npc_vehicles = []
    self.npc_vehicles.append( (lead_v, dest_lead_wp, lead_spawn) )
    #print(lead_v)

    #update interface
    self.create_navigation_section("Junction", self.crossings, "Prev Junction", "Next Junction", 1)
    self.create_navigation_section("Spawn", self.spawns, "Prev Spawn", "Next Spawn", 2)

#opens new window listing scenarios loaded from file
def open_navigator(self):
    self.navigator_window = tk.Toplevel(self.root)
    self.navigator_window.title("Select Scenario")

    # show current data
    self.label = tk.Label(self.navigator_window, text="", wraplength=400, justify="left")
    self.label.pack(pady=10)

    # current row number
    self.line_number_label = tk.Label(self.navigator_window, text="")
    self.line_number_label.pack(pady=5)

    # Pulsanti di navigazione
    self.prev_button = tk.Button(self.navigator_window, text="Prev", command=self.prev_line)
    self.prev_button.pack(side="left", padx=20)

    self.next_button = tk.Button(self.navigator_window, text="Next", command=self.next_line)
    self.next_button.pack(side="right", padx=20)

    # close button
    self.quit_button = tk.Button(self.navigator_window, text="Close window", command=self.quit_action)
    self.quit_button.pack(side="left", padx=10)
    #load current scenario
    self.load_button = tk.Button(self.navigator_window, text="Load Scenario", command=self.update_sim)
    self.load_button.pack(side="right", padx=10)

    # show first row
    self.update_scen_label()

