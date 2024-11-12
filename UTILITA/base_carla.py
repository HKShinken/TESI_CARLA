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

#funziona setta l'autopilota a true per le macchine create 
for v in world.get_actors().filter('*vehicle*'):
    v.set_autopilot(True)
