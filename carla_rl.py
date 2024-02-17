import glob
import os
import math
import random
import time
import numpy as np
import inspect
import cv2
import math
from collections import deque
from keras.applications.xception import Xception #serve pip install?
from keras.layers import Dense, GlobalAveragePooling2D
from keras.optimizers.legacy import Adam
from keras.models import Model
import carla
import tensorflow as tf
#import keras.backend.tensorflow_backend as backend #cambiato path di libreria con l'ultima versione di tf
import tensorflow.python.keras.backend as backend
from threading import Thread #non threading
from tqdm import tqdm
from keras.callbacks import TensorBoard


#size di default del sensore
IM_WIDTH = 500
IM_HEIGHT = 500

SECONDS_PER_EPISODE = 10
REPLAY_MEMORY_SIZE = 5_000 #5.000
MIN_REPLAY_MEMORY_SIZE = 1_000
MINIBATCH_SIZE = 16
PREDICTION_BATCH_SIZE = 1
TRAINING_BATCH_SIZE = MINIBATCH_SIZE // 4
UPDATE_TARGET_EVERY = 5 #AGGIORNAMENTO MODELLO OGNI 5 EPISODI
MODEL_NAME = "Xception"

#PER EVITARE CHE LA GPU ALLOCHI TUTTA LA MEMORIA
MEMORY_FRACTION = 0.6
MIN_REWARD = -200
DISCOUNT = 0.99
epsilon = 1
EPSILON_DECAY = 0.95 ##0.9975 ..
MIN_EPSILON = 0.001
EPISODES = 100

AGGREGATE_STATS_EVERY = 10 #10 EPISODES

#display camera da Carla
SHOW_PREVIEW = False


# Tensorboard custom per evitare di creare un file per ogni singolo frame ma per ogni episodio
class ModifiedTensorBoard(TensorBoard):

    # Overriding init to set initial step and writer (we want one log file for all .fit() calls)
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.step = 1
        self.writer = tf.summary.create_file_writer(self.log_dir)

    # Overriding this method to stop creating default log writer
    def set_model(self, model):
        pass

    # Overrided, saves logs with our step number
    # (otherwise every .fit() will start writing from 0th step)
    def on_epoch_end(self, epoch, logs=None):
        self.update_stats(**logs)

    # Overrided
    # We train for one batch only, no need to save anything at epoch end
    def on_batch_end(self, batch, logs=None):
        pass

    # Overrided, so won't close writer
    def on_train_end(self, _):
        pass

    # Custom method for saving own metrics
    # Creates writer, writes custom metrics and closes writer
    def update_stats(self, **stats):
        self._write_logs(stats, self.step)
		
    def _write_logs(self, logs, index):
        with self.writer.as_default():
           for name, value in logs.items():
               tf.summary.scalar(name, value, step=index)
               self.step += 1
               self.writer.flush()

class CarEnv:
    SHOW_CAM = SHOW_PREVIEW
    STEER_AMT = 1.0 #FULL TURN
    im_width = IM_WIDTH
    im_height = IM_HEIGHT
    front_camera = None
    
    def __init__(self):
        self.client = carla.Client("localhost", 2000)
        #self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.model = self.blueprint_library.find('vehicle.lincoln.mkz_2020')

    #reinizializza la classe con auto e sensori di default, svuotando le liste di attori
    def reset(self):
        self.collision_hist = [] # il sensore di collisione ritorna un set di eventi di collisione, es: se il paraurti tocca il suolo a causa di un dosso
        self.actor_list = []
        
        self.transform = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(self.model, self.transform)
        self.actor_list.append(self.vehicle)
        
        self.rgb_cam = self.blueprint_library.find('sensor.camera.rgb')
        self.rgb_cam.set_attribute('image_size_x', f"{self.im_width}")
        self.rgb_cam.set_attribute('image_size_y', f"{self.im_height}")
        self.rgb_cam.set_attribute("fov",f"110")
        
        #posizione sensori
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        
        #camera rgb
        self.sensor = self.world.spawn_actor(self.rgb_cam, transform, attach_to=self.vehicle)
        self.actor_list.append(self.sensor)
        self.sensor.listen(lambda data: self.process_img(data))
        
        #trick per rendere più reattiva l'auto
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))
        time.sleep(4)
        
        #definizione sensore di collisione
        colsensor = self.blueprint_library.find('sensor.other.collision')
        self.colsensor = self.world.spawn_actor(colsensor, transform, attach_to=self.vehicle)
        self.actor_list.append(self.colsensor)
        self.colsensor.listen(lambda event: self.collision_data(event))

        #controlla che la camera sia inizializzata, altrimeti attende
        while self.front_camera is None:
            time.sleep(0.01)

        self.episode_start = time.time() #in teoria vogliamo episodi di 10 secondi
        #trick per rendere più reattiva l'auto
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0))

        return self.front_camera
    
    #reinizializza la classe con oggetti e sensori di default, svuotando le liste di attrori e azioni
    def collision_data(self, event):
        self.collision_hist.append(event)

    #cattura i raw data del sensore e li trasforma in immagini priettate in imgshow di opencv
    def process_img(self, image):
        i = np.array(image.raw_data) #raw.data è un array flat
        #print(i.shape)
        i2 = i.reshape((self.im_height, self.im_width, 4))
        i3 = i2[:, :, :3] #intera altezza, larghezza, i primi tre valori sono rgb
        #print(i3)
        if self.SHOW_CAM:
            cv2.imshow("RGB_SENS",i3) #NON DARE UN TITOLO SE NO NON FUNZIONA
            cv2.waitKey(1) #ms: aggiornamento finestra a 0 non funziona
        self.front_camera = i3
        #return i3/255.0 #normalizzazione valori tra 0 e 1

    def step(self, action):
        if action == 0:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=-1*self.STEER_AMT)) #decrementa la quantità di sterzo verso sinistra
        elif action == 1:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0)) # dritto
        elif action == 2:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=1*self.STEER_AMT)) #incrementa la quantità di sterzo verso destra
        
        #se non collide 
        v = self.vehicle.get_velocity()
        #conversione a kilometri/h
        kmh = int(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2))

        #calcolo reward, controllo se c'è almeno una collisione
        if len(self.collision_hist) != 0:
            done = True
            reward = -200 #per ora usiamo questo valore
        elif kmh < 50:
            done = False
            reward = -1
        else: #SE 50 KMH AUMENTO REAWRD DI 1
            done = False
            reward = 1

        if self.episode_start + SECONDS_PER_EPISODE < time.time():
            done = True
        #controllo che la dimensione degli steps sia nei 10 seocndi
        return self.front_camera, reward, done, None


class DQNAgent:
    def __init__(self):
        self.model = self.create_model()
        self.target_model = self.create_model()
        self.target_model.set_weights(self.model.get_weights()) #aggiornamento #dopo n episodi sul train set si aggiorna il modello di riferimento
        
        self.replay_memory = deque(maxlen=REPLAY_MEMORY_SIZE) #da vedere sul corso: riprende la memoria di un azione precedente al fine di farci training

        self.tensorboard = ModifiedTensorBoard(log_dir=f"F:/models_carla_rl/logs/{MODEL_NAME}-{int(time.time())}")#da incollare, la funzione è ridotta per il nostro caso
        self.target_update_counter = 0 #aggiornato a ogni fine episodio
        self.graph = tf.compat.v1.get_default_graph() #queto da fare in un thread separato

        self.terminate = False #per terminare theads
        #dato che si usano dei thread servono dei flag di tracciamento modifiche
        self.last_logged_episode = 0 #per tracciare le modifiche sulla tensorboard da parte di un episodio
        self.training_initialized = False #per tracciare l'inizio della simulazione dato che uso dei thread

    def create_model(self):
        base_model = Xception(weights=None, include_top=False, input_shape=(IM_HEIGHT, IM_WIDTH, 3))

        x = base_model.output
        x = GlobalAveragePooling2D()(x)

        predictions = Dense(3, activation="linear")(x) #3 neuroni per le 3 opzioni di guida, sono le possibili predizioni
        model = Model(inputs=base_model.input, outputs=predictions) #istanziazione modello
        model.compile(loss="mse", optimizer=Adam(lr=0.001),metrics=["accuracy"]) #mse mean square error con learning rate 0.0001
        return model

    def update_replay_memory(self, transition): #transition contiene tutte le info per addestrare il modello
       # transition = (current_state, action, reward, new_state, done)
        self.replay_memory.append(transition)

    def train(self):
        if len(self.replay_memory) < MIN_REPLAY_MEMORY_SIZE: #controlla ci siano informazioni sufficienti per l'apprendimento
            return

        minibatch = random.sample(self.replay_memory, MINIBATCH_SIZE) #creazione di un minibatch col sample recuperato
        
        current_states = np.array([transition[0] for transition in minibatch])/255 #scaling dell'immagine
        #forse non serve with self.graph.as_default():
        current_qs_list = self.model.predict(current_states, PREDICTION_BATCH_SIZE) #predizione 

        new_current_states = np.array([transition[3] for transition in minibatch])/255 #scaling dell'immagine , [transition[3] + il prossimo stato
        #forse non serve with self.graph.as_default():
        future_qs_list = self.target_model.predict(new_current_states, PREDICTION_BATCH_SIZE) #predizione 
        
        X = [] #input
        y = [] #outputs
        
        #vogliamo enumerare i batch durante i training, creando Q value solo quando abbiamo future states
        # se non si crasha con l'auto settiamo Q su una formula
        for index, (current_state, action, reward, new_state, done) in enumerate(minibatch):
            if not done:
                max_future_q = np.max(future_qs_list[index])
                new_q = reward + DISCOUNT * max_future_q
            else: #se l'auto crasha
                new_q = reward
            
            current_qs = current_qs_list[index] #predizione corrente
            current_qs[action] = new_q          #azione per la predizione corrente
            
            X.append(current_state)
            y.append(current_qs)

        log_this_step = False
        if self.tensorboard.step > self.last_logged_episode:
            log_this_step = True
            self.last_log_episode = self.tensorboard.step

        #fit del modello con X e Y
        with self.graph.as_default():
            self.model.fit(np.array(X)/255, np.array(y), batch_size=TRAINING_BATCH_SIZE, verbose=0, shuffle=False, callbacks=[self.tensorboard] if log_this_step else None) #callback a tensorflow so sugli step significativi

        #aggiorno il contatore degli step utili all'apprendimento
        if log_this_step:
            self.target_update_counter += 1

        #copia del modello iniziale ogni tot aggiornamenti
        if self.target_update_counter > UPDATE_TARGET_EVERY:
            self.target_model.set_weights(self.model.get_weights())
            self.target_update_counter = 0

    def get_qs(self, state):
        return self.model.predict(np.array(state).reshape(-1, *state.shape)/255)[0] #immagine attuale
    
    #thread di training che attende la fine dell'addestramento
    def train_in_loop(self):
        #prima inizializzazione di input e output X, y
        X = np.random.uniform(size=(1, IM_HEIGHT, IM_WIDTH, 3)).astype(np.float32)
        y = np.random.uniform(size=(1,3)).astype(np.float32)
		
		#forse non serve dato che va in modalità eager
        #with self.graph.as_default():
        #    self.model.fit(X,y, verbose=False, batch_size=1)
        self.model.fit(X,y, verbose=False, batch_size=1)

        self.training_initialized = True

        while True:
            if self.terminate: #attende terminazione flag thread altrimenti continua l'addestramento
                return
            self.train()
            time.sleep(0.01)

if __name__ == '__main__':
    FPS = 60 #massimo...
    ep_rewards = [-200]
    random.seed(1)
    np.random.seed(1)
    #old: tf.set_random_seed(1)
    tf.random.set_seed(1)
    #old: gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=MEMORY_FRACTION) #masimo 60% per modello
    gpu_options = tf.compat.v1.GPUOptions(per_process_gpu_memory_fraction=MEMORY_FRACTION) #masimo 60% per modello
    backend.set_session(tf.compat.v1.Session(config=tf.compat.v1.ConfigProto(gpu_options=gpu_options)))
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
    
	#creazione path per memorizzazione del modello
    if not os.path.isdir("F:\models_carla_rl"):
        os.makedirs("F:\models_carla_rl")
    
	#inizializza agente
    agent = DQNAgent()
	#setta gli attori di carla
    env = CarEnv()
	
    #inizializza il thread di training e apetta che si inizializzi
    trainer_thread = Thread(target=agent.train_in_loop, daemon=True)
    trainer_thread.start()
    print("ciao prima")
    while not agent.training_initialized:
        time.sleep(0.01)
    print("ciao") 
	#inizializzazione predizioni																								 
    agent.get_qs(np.ones((env.im_height, env.im_width, 3)))
    #iterazione episodi
    for episode in tqdm(range(1, EPISODES+1), ascii=True, unit="episodes"):
        env.collision_hist = []
        #aggiornamento tensorboard a ogni episodio (no ad ogni frame)
        agent.tensorboard.strp = episode
        #inizializza un nuovo episodio
        episode_reward = 0
        step = 1
        #resetta carla allo stato iniziale con una  nuova macchina
        current_state = env.reset()
        #resetta il flag dell' episodio, sarà true a episodio terminato
        done = False
        episode_start =  time.time()
        
        while True:
            if np.random.random() > epsilon: #con eps alto ci sarebbero molte azioni random
				#prende un azione dalla Q table								
                action = np.argmax(agent.get_qs(current_state))
            else:#altrimenti azione random
                action = np.random.randint(0,3) #azione casuale tra le tre disponibili
				#crea un attesa prima del prossimo frame					   
                time.sleep(1/FPS)
            
            new_state, reward, done, _ = env.step(action)
            episode_reward += reward
			
            #aggiornamento memoria con target model 							
            agent.update_replay_memory((current_state, action, reward, new_state, done))
			
			#aggiorno lo stato attuale con quello ritornato dall'ultimo step
            current_state = new_state																				
            step += 1
            
            if done: #se la funzione step ritorna almeno una collisione fermo l'episodio
                break
            
		#fine episodio, distrugge l'agente			
        for actor in env.actor_list:
            actor.destroy()
        
        # memorizza il reward dell'episodio e memorizza le stats
        #max, min, avg reward
        ep_rewards.append(episode_reward)
        if not episode % AGGREGATE_STATS_EVERY or episode == 1:
            average_reward = sum(ep_rewards[-AGGREGATE_STATS_EVERY:])/len(ep_rewards[-AGGREGATE_STATS_EVERY:])
            min_reward = min(ep_rewards[-AGGREGATE_STATS_EVERY:])
            max_reward = max(ep_rewards[-AGGREGATE_STATS_EVERY:])
            agent.tensorboard.update_stats(reward_avg=average_reward, reward_min=min_reward, reward_max=max_reward, epsilon=epsilon)
        print('STAMPO MIN REWARD CALCOLATO')
        print(min_reward)
         # Save model, but only when min reward is greater or equal a set value
        if min_reward >= MIN_REWARD:
            agent.model.save(f'F:/models_carla_rl/models/{MODEL_NAME}__{max_reward:_>7.2f}max_{average_reward:_>7.2f}avg_{min_reward:_>7.2f}min__{int(time.time())}.model')
        
        # Decay epsilon
        if epsilon > MIN_EPSILON:
            epsilon *= EPSILON_DECAY
            epsilon = max(MIN_EPSILON, epsilon)
    
    # termina l'addestramento per il thread e lo chiude
    agent.terminate = True
    trainer_thread.join()
    agent.model.save(f'F:/models_carla_rl/models/{MODEL_NAME}__{max_reward:_>7.2f}max_{average_reward:_>7.2f}avg_{min_reward:_>7.2f}min__{int(time.time())}.model')
            
        

    
