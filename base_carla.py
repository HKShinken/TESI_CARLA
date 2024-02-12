import carla
import math
import random
import time

#riferimento al tutorial
#https://carla.readthedocs.io/en/latest/tuto_first_steps/
#https://www.youtube.com/watch?v=pONr1R1dy88

client = carla.Client('localhost', 2000)
#accesso agli asset della simulazione
world = client.get_world()

#serve  per creare oggetti nella simulazione tipo veicoli, caos, pedoni.
bp_lib = world.get_blueprint_library()

#punti per spawnare oggetti
spawn_points = world.get_map().get_spawn_points();

vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
#il try serve per trovare locazioni libere altrimenti fallirebbe
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

#per trovare il veicolo serve una telecamera che si può spostare in una posizione specifica
spectator = world.get_spectator()
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation)
spectator.set_transform(transform)

#genera 30 macchine casuali, sempre provando spawn points liberi
for i in range(30):
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

#il traffic manager può essere usato sia per cambiare il comportamento dei veicoli che per rilevare collisioni
#collision_detection(self, reference_actor, other_actor, detect_collision)
tm = client.get_trafficmanager(8000)
	
# tramite traffic manager, setta l' autopilota o le luci per tutti i veicoli presenti in simulazione 
#fa fare incidenti
#for v in world.get_actors().filter('*vehicle*'):
#    v.set_light_state(carla.VehicleLightState.RightBlinker)


#funziona setta l'autopilota a true per le macchine create 
for v in world.get_actors().filter('*vehicle*'):
    v.set_autopilot(True)

#settaggio di sensori (gps, collisioni ecc..), in questo caso camera rgb
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(z=2)) #posizione sensore
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

#QUESTO PER SEGUIRE IL VEICOLo ogni 4 secondi
while True:
    time.sleep(5)
    spectator.set_transform(camera.get_transform())

#registrazione su png files
camera.listen(lambda image: image.save_to_disk('F:/out/%06d.png' % image.frame))

camera.close()
