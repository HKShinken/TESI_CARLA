import carla
import random
import math
import threading
import lib_spawn as spb
import lib_map_draw as dr
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
    for wp2_coords, pairs in coord_groups.items():
        if len(pairs) > 1:
            print(f"Le coordinate {wp2_coords} accomunano le seguenti coppie:")
            for pair in pairs:
                print(p_wp(pair[0]) + ", " + p_wp(pair[1]))
            print()
        
    return coord_groups


#routine che per ogni veicolo in input avvia un agente in un thread dedicato
#agent type  cautious, normal, aggressive
def roam_vehicle(in_vehicle, dest_wp,  visibile=0, agent_behaviour='normal'):
    
    #wp corrispondente allo spawn del veicolo
    start_location = in_vehicle.get_location()
    start_waypoint = world.get_map().get_waypoint(start_location)
    
    #L' AGENTE BEHAVIOUR REAGISCE A SEMAFORI E SEGNALI DI STOP, EVITA PEDONI, SEGUE LE MACCHINE 
    #3 POSSIBILI PROFILI AL MOMENTO: cautious, normal, aggressive -> https://carla.readthedocs.io/en/0.9.12/adv_agents/#behavior-types
    agent = BehaviorAgent(in_vehicle, behavior=agent_behaviour)
    
    #recupera in forma di lista il percorso tra inizio e destinazione
    route = agent.trace_route(start_waypoint, dest_wp)

    #TEMPORANEO: IGNORA I SEMAFORI
    agent.ignore_traffic_lights(True)
    
    agent.set_destination(dest_wp.transform.location)
    #print(dest_wp)
    curr_wp = None
    global stop_threads
    #main loop
    while True:
        #controlla se sorpassa un rosso
        if spb.has_crossed_red_light(in_vehicle, world) > 0:
            print("ATTENZIONE, SEMAFORO ROSSO SUPERATO!")
        #tolleranza media di distanza per ora 3,7
        if agent.done() or spb.wp_distance(curr_wp, dest_wp) <= 3.7 or stop_threads:
            print(f"Il veicolo {in_vehicle} ha raggiunto la sua destinazione (Stop forzato={stop_threads})")
            in_vehicle.destroy()
            break
        #a ogni iterazione applica il controllo dell'agent
        in_vehicle.apply_control(agent.run_step())
        curr_wp = world.get_map().get_waypoint(in_vehicle.get_location())
        
        if visibile == 1:
            draw_symbol(curr_wp, 0.01, 'NPC', 255, 255, 255) 
            draw_symbol(dest_wp, 0.01, 'DESTINAZIONE', 255, 255, 255)

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
    for wp2_coords, pairs in coord_groups.items():
        if len(pairs) > 1:
            print(f"Le coordinate {wp2_coords} accomunano le seguenti coppie:")
            for pair in pairs:
                print(p_wp(pair[0]) + ", " + p_wp(pair[1]))
            print()
        
    return coord_groups  

def p_wp(w):
  return f"({round(w.transform.location.x,3)}) - ({round(w.transform.location.y,3)})"
##############################################      MAIN CODE      ############################################

##############Connessione al simulatore################
'''
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
bp_lib = world.get_blueprint_library() 
map = world.get_map()

# lista di wp alternati (es.: ogni 2 metri)
waypoints = map.generate_waypoints(2.0)

# array di incroci contenente coppie (id, oggetto_incrocio)
unique_junctions = []

#######################################################

# lettura dei wp eventualmente presenti ad un' intersezione
#N.B.: più waypoint possono appartenere a uno stesso incrocio
for waypoint in waypoints:
    if waypoint.is_junction:
        junction = waypoint.get_junction()
        if not(any(u[0] == junction.id for u in unique_junctions)):
            unique_junctions.append( (junction.id, junction))

# Stampa il numero di incroci (9 distinti in town 10)
print(f"Numero di incroci trovati: {len(unique_junctions)}\n")

print(unique_junctions)

#questa è interessante perchè si scontrano senza semafori unique_junctions[3][1]
#la prima cella è l'iesimo elemento dell'array fatto di coppie (id, intersezione)
#la seconda cella lasciala a 1, contiene la lista di coppie
#la penultima indicizza la singola coppia 
# l'ultima lasciala così elenca il primo elemento di ciascuna coppia
wp_pairs = unique_junctions[3][1].get_waypoints(carla.LaneType.Driving)
wp1 = wp_pairs[4][0]

#per ora wp pairs è una lista presa a caso
grouped_pairs = group_pairs_by_second_waypoint(wp_pairs)

spawned_vehicles = []
# Stampa i gruppi di coppie per ciascun secondo waypoint
for wp2_coords, pairs in grouped_pairs.items():

    if len(pairs) > 1:
       draw_intersection(pairs)
       spawned_vehicles += (spb.spawn_all_behind_wp_couple(world, pairs, distance=10, focus=1))

print(spawned_vehicles)

stop_threads = False
threads = []

# qui bisogna impostare l'agent per ogni veicolo
#fai debug della funzione e cerca di capire dal codice come si lascia i semafori alle spalle
for v, destwp in spawned_vehicles:
    if v is not None:
        #v.set_autopilot(True)
        thread = threading.Thread(target=roam_vehicle, args=(v,destwp,1))
        threads.append(thread)
        thread.start()
'''