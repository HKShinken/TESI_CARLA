import carla
import math

#alignment treshold
tolerance = 2
#time for drawing symbols on simulator
draw_time = 30

#draws path between two waypoints, blue if point are aligned,red if there is a curve
def draw_route(world, wp1, wp2, symbol, r, g, b, draw_time, limit=99999):
    distance = 3.0  
    symbol = str(symbol)

    wp1 = wp1.next(distance)[0] #gets next waypoint at set distance
    
    # Draws colored symbols on each waypoit between the path
    while wp1.transform.location.distance(wp2.transform.location) > distance and limit > 0:
        world.debug.draw_string(wp1.transform.location, symbol, draw_shadow=True,
                                color=carla.Color(r, g, b), life_time = draw_time, persistent_lines=True)
        # Vai al prossimo waypoint
        wp1 = wp1.next(distance)[0]
        limit = limit -1


#draws specific symbol on diven wayopint
def draw_symbol(world, wp, duration, symbol, r, g, b):
    world.debug.draw_string(wp.transform.location, symbol, draw_shadow=False, color=carla.Color(r, g, b), life_time=duration)

def draw_spawn(world, sp, duration, symbol, r, g, b):
    world.debug.draw_string(sp.location, symbol, draw_shadow=True, color=carla.Color(r, g, b), life_time=duration)

# draws a path between each couple of waypoint present in the input list pair
def draw_intersection(world, pairs, draw_time):

    global pair_counter #per modificare variabile globale
    for (wp1, wp2) in pairs:
       
        x1, y1 = wp1.transform.location.x, wp1.transform.location.y
        x2, y2 = wp2.transform.location.x, wp2.transform.location.y

        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        
        # check if are aligned e set the symbol properly
        if dx <= tolerance or  dy <= tolerance:
            draw_route(world, wp1, wp2, '<->', 0, 0, 255, draw_time)
        else:
            draw_route(world, wp1, wp2, '|', 255, 0, 0, draw_time )

#Routine thaf focuses the specator above the input location loc
def focus_above(world, loc, z_height=60, in_pitch=-90):
    camera_location = carla.Location(loc.x, loc.y, z=z_height) #altezza 60
    camera_rotation = carla.Rotation(pitch=in_pitch)  # Orientamento verso il basso
    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(camera_location, camera_rotation))

#print more human readable coordinate, truncated at third decimal
def p_wp(w):
  return f"({round(w.transform.location.x,3)}) - ({round(w.transform.location.y,3)})"


        