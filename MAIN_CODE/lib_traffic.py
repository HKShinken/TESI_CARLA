import carla
import math
import random
import threading
from collections import defaultdict
import lib_map_draw as dr
import lib_spawn as spb
from importlib import reload
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.behavior_agent import BehaviorAgent
reload(dr)
reload(spb)

# shared event to force thread stop
stop_event = threading.Event()

def stop_threads():
    stop_event.set()  # set stop thread flag

def reset_stop_event():
    stop_event.clear()

#routine that starts a thread for each driving agent
def roam_vehicle(world, in_vehicle, dest_wp, symbol, visibile=0, agent_behaviour='normal', disable_trf = False):
    
    #wp of vehicle
    start_location = in_vehicle.get_location()
    start_waypoint = world.get_map().get_waypoint(start_location)
    
    #AGENT REACTS TO TRAFFIC LIGHTS, STOP, TRIES TO AVOID COLLISION WITH PEDESTRIANS AND VEHICLES.. 
    #3 PROFILES: cautious, normal, aggressive -> https://carla.readthedocs.io/en/0.9.12/adv_agents/#behavior-types
    agent = BehaviorAgent(in_vehicle, behavior=agent_behaviour)
    
    #retrives shortest path between wp1 and wp2
    route = agent.trace_route(start_waypoint, dest_wp)

    #skip sem if disable_trf=True
    agent.ignore_traffic_lights(disable_trf)
    
    agent.set_destination(dest_wp.transform.location)
    
    curr_wp = None
    global stop_threads

    while True:
        #controlla se sorpassa un rosso
        #if spb.has_crossed_red_light(in_vehicle, world) > 0:
            #print("ATTENZIONE, SEMAFORO ROSSO SUPERATO!")
        #tolerance from destination 3,7
        if agent.done() or spb.wp_distance(curr_wp, dest_wp) <= 3.7 or stop_event.is_set():
            print(f"Vehicle {in_vehicle} has reached its destination (Forced stop={stop_event.is_set()})")
            in_vehicle.destroy()
            break
        #for each iteration applies agentt controls
        in_vehicle.apply_control(agent.run_step())
        curr_wp = world.get_map().get_waypoint(in_vehicle.get_location())

        r, g, b = 255, 255, 255;
        if symbol == 'EGO_VEHICLE':
          g,b = 0,0
        
        if visibile >= 1:
            dr.draw_symbol(world, curr_wp, 0.01, symbol, r, g, b) 
        if visibile == 2:
            dr.draw_symbol(world, dest_wp, 0.01, f"DESTINAZIONE_{symbol}", r, g, b)


#starts vehicles in list,towards each respective wp
def start_vehicle_list(world, spawned_vehicles, symbol, visible = 1, behaviour = 'normal', disable_trf = False):
    stop_threads = False
    threads = []
    for v, destwp, sp in spawned_vehicles:
        if v is not None and destwp is not None:
            thread = threading.Thread(target=roam_vehicle, args=(world, v, destwp, symbol, visible, behaviour.lower(), disable_trf))
            threads.append(thread)
            thread.start()
    return threads;

#starts single vehicle, towards its respective wp
def start_vehicle(world, spawned_vehicle, symbol, visible = 1, behaviour = 'normal', disable_trf = False):
    stop_threads = False
    thread = None
    v, destwp, sp= spawned_vehicle
    if destwp is not None and destwp is not None:
        thread = threading.Thread(target=roam_vehicle, args=(world, v, destwp, symbol, visible, behaviour.lower(), disable_trf))
        thread.start()
    return thread;
        
