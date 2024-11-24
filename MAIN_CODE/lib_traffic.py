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

stop_threads = False

#routine che per ogni veicolo in input avvia un agente in un thread dedicato
#agent type  cautious, normal, aggressive
def roam_vehicle(world, in_vehicle, dest_wp, symbol, visibile=0, agent_behaviour='normal'):
    
    #wp of vehicle
    start_location = in_vehicle.get_location()
    start_waypoint = world.get_map().get_waypoint(start_location)
    
    #AGENT REACTS TO TRAFFIC LIGHTS, STOP, TRIES TO AVOID COLLISION WITH PEDESTRIANS AND VEHICLES.. 
    #3 PROFILES AL MOMENTO: cautious, normal, aggressive -> https://carla.readthedocs.io/en/0.9.12/adv_agents/#behavior-types
    agent = BehaviorAgent(in_vehicle, behavior=agent_behaviour)
    
    #retrives shortest path between wp1 and wp2
    route = agent.trace_route(start_waypoint, dest_wp)

    #wip TEMPORANEO: IGNORA I SEMAFORI
    agent.ignore_traffic_lights(True)
    
    agent.set_destination(dest_wp.transform.location)
    #print(dest_wp)
    curr_wp = None
    global stop_threads

    while True:
        #controlla se sorpassa un rosso
        #if spb.has_crossed_red_light(in_vehicle, world) > 0:
            #print("ATTENZIONE, SEMAFORO ROSSO SUPERATO!")
        #tolerance from destination 3,7
        if agent.done() or spb.wp_distance(curr_wp, dest_wp) <= 3.7 or stop_threads:
            print(f"Il veicolo {in_vehicle} ha raggiunto la sua destinazione (Stop forzato={stop_threads})")
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


#avvia i veicoli in lista direzionandoli verso il rispettivo wp di destinazione
def start_vehicle_list(world, spawned_vehicles, symbol, visible = 1, behaviour = 'normal'):
    stop_threads = False
    threads = []
    for v, destwp in spawned_vehicles:
        if v is not None and destwp is not None:
            thread = threading.Thread(target=roam_vehicle, args=(world, v, destwp, symbol, visible, behaviour.lower()))
            threads.append(thread)
            thread.start()
    return threads;

#avvia il singolo veicolo direzionandolo verso il wp di destinazione
def start_vehicle(world, spawned_vehicle, symbol, visible = 1, behaviour = 'normal'):
    stop_threads = False
    thread = None
    print(f"in start {spawned_vehicle}")
    v = spawned_vehicle[0]
    destwp = spawned_vehicle[1]
    if destwp is not None and destwp is not None:
        thread = threading.Thread(target=roam_vehicle, args=(world, v, destwp, symbol, visible, behaviour.lower()))
        thread.start()
    return thread;
        
