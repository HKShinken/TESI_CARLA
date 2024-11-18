import carla
from collections import defaultdict

#tolleranza di allineamento tra coppie su stessa lane di un incrocio 
tolerance = 2
#contatore di tutte le coppe di wp
pair_counter = 0
#ausiliaria
pair_indexer = defaultdict(int)

#ausiliarie per disegnare simboli sulla mappa
#colora il percorso tra due waypoint, bianco se i punto sono allineati sulla stessa trada, rosso se c'è una curva
def draw_route(world, wp1, wp2, symbol, r, g, b):
    distance = 3.0  # Distanza tra i waypoint intermedi in metri
    symbol = str(symbol)

    wp1 = wp1.next(distance)[0]
    
    # Disegna simboli lungo il percorso fino al waypoint di destinazione
    while wp1.transform.location.distance(wp2.transform.location) > distance:
        # Disegna il simbolo nel waypoint corrente
        world.debug.draw_string(wp1.transform.location, symbol, draw_shadow=False,
                                color=carla.Color(r, g, b), life_time=60.0, persistent_lines=True)
        # Vai al prossimo waypoint
        wp1 = wp1.next(distance)[0]


#disegna due x rosse sui wp
def draw_couple(world, idx, wp1, wp2):
    # Disegna una 'X' su wp1 con il numero progressivo
    world.debug.draw_string(wp1.transform.location, f"X {idx}", 
                            draw_shadow=True, 
                            color=carla.Color(r=0, g=255, b=0), 
                            life_time=60.0, 
                            persistent_lines=True)

    # Disegna una 'X' su wp2 con lo stesso numero progressivo
    world.debug.draw_string(wp2.transform.location, f"X {idx}", 
                            draw_shadow=True, 
                            color=carla.Color(r=255, g=0, b=0), 
                            life_time=60.0, 
                            persistent_lines=True)

#disegna una stringa in uno specifico waypoint
def draw_symbol(world, wp, duration, symbol, r, g, b):
    world.debug.draw_string(wp.transform.location, symbol, draw_shadow=False, color=carla.Color(r, g, b), life_time=duration)

# Funzione ausiliaria che disegna il percorso tra ogni coppia di wp presenti nella lista in input
def draw_intersection(world, pairs):

    global pair_counter #per modificare variabile globale
    
    for (wp1, wp2) in pairs:
       
        # Ottieni le coordinate x e y dei waypoints
        x1, y1 = wp1.transform.location.x, wp1.transform.location.y
        x2, y2 = wp2.transform.location.x, wp2.transform.location.y

        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        
        # Controllo se sono sullo stesso asse x o y
        if dx <= tolerance or  dy <= tolerance:
            #print(f"Pair {idx}: I waypoints sono allineati ({wp1.transform.location.x} - {wp1.transform.location.y}) - ({wp2.transform.location.x} - {wp2.transform.location.y})")
            draw_couple(world,pair_counter, wp1, wp2)
            draw_route(world, wp1, wp2, pair_counter, 0, 0, 255)
        else:
            # altrimenti c'è un angolo tra i due wp
            #print(f"Pair {idx}: tra i waypoints c'è una curva ({wp1.transform.location.x} - {wp1.transform.location.y}) - ({wp2.transform.location.x} - {wp2.transform.location.y})")
            draw_couple(world, pair_counter, wp1, wp2)
            draw_route(world, wp1, wp2, pair_counter, 255, 255, 255 )
            
        pair_indexer[(wp1, wp2)] = pair_counter;
        pair_counter = pair_counter + 1

#Funzione ausiliaria per porre il focus spettatore dall'alto
def focus_above(world, loc, z_height=60, in_pitch=-90):
    camera_location = carla.Location(loc.x, loc.y, z=z_height) #altezza 60
    camera_rotation = carla.Rotation(pitch=in_pitch)  # Orientamento verso il basso
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(camera_location, camera_rotation))

#funzione di stampa dei wp
def p_wp(w):
  return f"({round(w.transform.location.x,3)}) - ({round(w.transform.location.y,3)})"