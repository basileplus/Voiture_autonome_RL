# Python imports
import gym as gym
import numpy as np
import time
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import struct

import sys
#sous Linux, chemin a adapter export WEBOTS_HOME=/usr/local/webots
sys.path.append('/usr/local/webots/lib/controller/python/')
sys.path.append('/home/basile/Documents/TER_basile/Simulateur_CoVAPSy_Webots2023b_RL_essai3/controllers')



	


from vehicle import Driver

# Global constants
RECEIVER_SAMPLING_PERIOD = 64	# In milliseconds
PI = 3.141592653589793
MAXSPEED = 12 #km/h
VITESSE_MAX_M_S = MAXSPEED/3.6
MAXANGLE_DEGRE = 18
RESET_STEP = 16384	# Number of steps between 2 forced resets to avoid overfitting


# Custom Gym environment
class WebotsGymEnvironment(Driver, gym.Env):
	def __init__(self):
		super().__init__()            
		#valeur initiale des actions
		self.consigne_angle = 0.0 #en degres
		self.consigne_vitesse = 0.1 #en m/s
		#compteur servant à la supervision de l'apprentissage
		self.numero_crash = 0         # compteur de collisions
		self.nb_pb_lidar = 0         # compteur de problèmes de communication avec le lidar
		self.nb_pb_acqui_lidar=0    # compteur de problèmes d'acquisition lidar (il renvoie 0 ce qui n'est pas possible)
		self.nb_demarrage_lidar=0        # compteur de démarrages de lidar (il faut parfois le démarrer plusieurs fois)
		self.reset_counter = 0         # compteur de pas d'apprentissage pour arrêter un épisode après RESET_STEP pas
		self.packet_number = 0         # compteur de messages envoyés

		# Emitter / Receiver for reset signal
		self.ResetEmitter = super().getDevice("ResetEmitter")
		self.ResetReceiver = super().getDevice("ResetReceiver")
		self.ResetReceiver.enable(RECEIVER_SAMPLING_PERIOD)
		self.ResetReceiver.setChannel(2)
		self.ResetEmitter.setChannel(1)

		# Receiver for lap and sector times
		self.LapReceiver = super().getDevice("LapReceiver")
		self.LapReceiver.enable(RECEIVER_SAMPLING_PERIOD)
		self.LapReceiver.setChannel(3)

		# Lidar initialisation
		self.lidar = super().getDevice("RpLidarA2")
		self.lidar.enable(int(super().getBasicTimeStep()))
		self.lidar.enablePointCloud()
		self.lap_time = 0
		self.sector1_t = 0
		self.sector2_t = 0
		self.sector3_t = 0

		# Action space
		self.action_space = gym.spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
		# Observation space
		#self.observation_space = gym.spaces.Box(np.ones(360)*-1, np.ones(360), dtype=np.float64)
		
		self.observation_space = gym.spaces.Dict({
		"current_lidar":gym.spaces.Box(np.zeros(201), np.ones(201), dtype=np.float64),
		"previous_lidar":gym.spaces.Box(np.zeros(201), np.ones(201), dtype=np.float64),
		"previous_speed":gym.spaces.Box(np.array([0]), np.array([1]), dtype=np.float64),
		"previous_angle":gym.spaces.Box(np.array([-1]), np.array([1]), dtype=np.float64),
		})
		
	def get_lidar_mm(self):
		tableau_lidar_mm = np.zeros(360)
		try :
			donnees_lidar_brutes = np.array(self.lidar.getRangeImage()) #lidar en mm
			for i in range(0,101) :
				if (donnees_lidar_brutes[-i]>0) and (donnees_lidar_brutes[-i]<12) :
					tableau_lidar_mm[i] = 1000*donnees_lidar_brutes[-i]
				else :
					tableau_lidar_mm[i] = 0
			for i in range(101,260) : #zone correspondant à l'habitacle de la voiture
				tableau_lidar_mm[i] = 0    
			for i in range(260,360) :
				if (donnees_lidar_brutes[-i]>0) and (donnees_lidar_brutes[-i]<12) :
					tableau_lidar_mm[i] = 1000*donnees_lidar_brutes[-i]
				else :
					tableau_lidar_mm[i] = 0
		except : 
			self.nb_pb_lidar+=1
			print("souci lidar"+str(self.nb_pb_lidar))
		return tableau_lidar_mm
	
	def set_vitesse_m_s(self,vitesse_m_s):
		speed = vitesse_m_s*3.6
		if speed > MAXSPEED :
			speed = MAXSPEED
		if speed < 0 :
			speed = 0
		super().setCruisingSpeed(speed)
	
	def set_direction_degre(self,angle_degre):
		if angle_degre > MAXANGLE_DEGRE:
			angle_degre = MAXANGLE_DEGRE
		elif angle_degre < -MAXANGLE_DEGRE:
			angle_degre = -MAXANGLE_DEGRE   
		angle = -angle_degre * PI/180
		super().setSteeringAngle(angle)


	# Get Lidar observation
	def get_observation(self,init=False):
		tableau_lidar_mm = self.get_lidar_mm()
		i = 0
		while(tableau_lidar_mm[0] == 0) and (tableau_lidar_mm[1] == 0) and (i<20) :
			#on essaie d'avoir un tableau de valeur correct, 20 essais max
			self.nb_pb_acqui_lidar+=1
			# print("souci d'aquisition lidar"+str(self.nb_pb_acqui_lidar))
			i = i+1
			super().step()
			tableau_lidar_mm = self.get_lidar_mm() #lidar en mm
  	
		if init:
			current_lidar=(np.concatenate((tableau_lidar_mm[0:101],tableau_lidar_mm[260:360]),axis=None)).astype("float64")/12000
			previous_lidar=(np.concatenate((tableau_lidar_mm[0:101],tableau_lidar_mm[260:360]),axis=None)).astype("float64")/12000
			previous_speed=np.zeros(1)
			previous_angle=np.zeros(1)

		else:
			#grandeurs normalisées pour observation
			previous_lidar=self.observation["current_lidar"]
			current_lidar=(np.concatenate((tableau_lidar_mm[0:101],tableau_lidar_mm[260:360]),axis=None)).astype("float64")/12000
			previous_speed=np.array([self.consigne_vitesse/VITESSE_MAX_M_S])
			# si on a un capteur de vitesse sur la voiture relle : 
			# previous_speed=[(super().getTargetCruisingSpeed()/3.6)/VITESSE_MAX_M_S]
			previous_angle=np.array([self.consigne_angle/MAXANGLE_DEGRE])
			# si on a un capteur de vitesse sur la voiture relle : 
			# previous_speed=[(super().getSteeringAngle()*180/PI)/MAXANGLE_DEGRE]
 
		observation = {
                        "current_lidar": current_lidar,
                        "previous_lidar":previous_lidar,
                        "previous_speed":previous_speed,
                        "previous_angle":previous_angle,
                      }
		# print(observation["current_lidar"])
		self.observation=observation
		return observation
		
	
	# Reward function
	def get_reward(self, obs):
		reward = 0
		done = False
		mini = 1

		# Get lap and sector times
		while(self.LapReceiver.getQueueLength() > 0) : 
			data = self.LapReceiver.getBytes()
			print("tt02 received : ", data)
			self.sector1_t, self.sector2_t, self.sector3_t, self.lap_time = struct.unpack("ffff", data) # Unpack the data
			self.LapReceiver.nextPacket()

		
		#recherche de la distance la plus faible mesuree par le lidar entre -40 et +40°
		for i in range(-40,40) :
			if (obs["current_lidar"][i] < mini and obs["current_lidar"][i]!=0) :
				mini = obs["current_lidar"][i]
		
		#si le lidar indique 0 sur 3 valeurs devant, c'est qu'il y a un souci
		if obs["current_lidar"][0]==0 and obs["current_lidar"][1]==0 and obs["current_lidar"][-1]==0 :
			# Crash
			self.numero_crash += 1
			print("crash num " + str(self.numero_crash) + " pb acqui lidar ")
			reward = -300 
			done = True
		
		#si la distance au mur est inférieure à 168mm, c'est que la voiture est en collision avec
		elif mini < 0.014 : #0.014 <-> 168 mm
			# Crash
			self.numero_crash += 1
			print("crash num " + str(self.numero_crash))
			reward = -300
			done = True
		
		#si la voiture va trop vite (chute dans le vide en dehors du sol de la piste), on redémarre
		elif super().getCurrentSpeed() > 30.0 : #si dysfonctionnement
			# Crash
			self.numero_crash += 1
			print("crash num " + str(self.numero_crash) + " vitesse trop grande")
			reward = -300 
			done = True	
		
		else:
			
			for sector_t in [self.sector1_t, self.sector2_t, self.sector3_t]:

				if sector_t > 0 :
					print("reward for sector_t "+str(sector_t))
					reward += 10*1/sector_t
			if self.lap_time > 0 :
				reward += 100*1/self.lap_time 
			#reward = 12 * (mini-0.014) + 1 * super().getTargetCruisingSpeed() # Récompense pour une grande distance aux obstacles et une grande vitesse
			#print("reward : "+str(reward))

		#Reset si la voiture a fait beaucoup de pas
		self.reset_counter += 1
		if self.reset_counter % RESET_STEP == 0:
			print("Reset") 
			done = True

		return reward, done		

	# Reset the simulation
	def reset(self):
		
		# Reset speed and steering angle et attente de l'arrêt de la voiture
		self.consigne_vitesse = 0.0
		self.consigne_angle = 0.0
		self.set_vitesse_m_s(self.consigne_vitesse )
		self.set_direction_degre(self.consigne_angle)
		for i in range(20) :
				super().step()	
		self.reset_counter = 0
		
		
		if(self.numero_crash != 0):         
			#attente de l'arrêt de la voiture
			while abs(super().getCurrentSpeed()) >= 0.001 :    		           	
				print("voiture pas encore arrêtée")
				super().step()
            		
			# Return an observation
			self.packet_number += 1
			# Envoi du signal de reset au Superviseur pour replacer les voitures
			message = "voiture crash envoi numero " + str(self.packet_number)
			print("tt02 sends : " + message)
			self.ResetEmitter.send(message)
			super().step()
			#attente de la remise en place des voitures
			while(self.ResetReceiver.getQueueLength() == 0) : 
				self.set_vitesse_m_s(self.consigne_vitesse )
				super().step()

			data = self.ResetReceiver.getString()
			print("tt02 received : " + data)
			self.ResetReceiver.nextPacket()
			# print(data) #affichage du message reçu du spuerviseur
			super().step()

		#quelques pas de simulations pour que le lidar fasse 2 tours (200 ms) avant la première observation
		mini=130000
		i=0
		while((mini <= 120) or (mini == 130000)) and (i<200) :
			#on essaie d'avoir un tableau de valeur correct (au moins 170 mm par mesure), 20 essais max
			super().step()
			tableau_lidar_mm = self.get_lidar_mm()
			self.nb_demarrage_lidar +=1
			i = i+1
			for j in range (-40,+40) :
				if tableau_lidar_mm[j] < mini :
					mini = tableau_lidar_mm[j]
		# print("démarrage lidar num "+str(self.nb_demarrage_lidar)+ " mini = " +str(mini))
		return self.get_observation(True)
	
	# Step function
	def step(self, action):
		current_speed = self.consigne_vitesse
		current_angle = self.consigne_angle
		self.consigne_vitesse=(current_speed+action[0]*0.05)
		self.consigne_angle=(current_angle+action[1]*9.0)
		
		# saturations
		if self.consigne_angle > MAXANGLE_DEGRE  :
			self.consigne_angle = MAXANGLE_DEGRE 
		elif self.consigne_angle < -MAXANGLE_DEGRE  :
			self.consigne_angle = -MAXANGLE_DEGRE 
		
	
		if self.consigne_vitesse > VITESSE_MAX_M_S :
			self.consigne_vitesse = VITESSE_MAX_M_S
		if self.consigne_vitesse < 0.1:
			self.consigne_vitesse = 0.1
			
		self.set_vitesse_m_s(self.consigne_vitesse)
		self.set_direction_degre(self.consigne_angle)
		super().step()
		
		obs = self.get_observation()
		reward, done = self.get_reward(obs)
		return obs, reward, done, {}


# Main function
def main():
	t0 = time.time()
	env = WebotsGymEnvironment()
	print("environnement créé")
	check_env(env)
	print("vérification de l'environnement")
	
	# Model definition
	model = PPO(policy="MultiInputPolicy",
		env=env, 
		learning_rate=5e-4,
		verbose=1,
		device='cpu',
		tensorboard_log='./PPO_Tensorboard',)
			# Additional parameters
		# n_steps=2048,
		# batch_size=64, # Factor of n_steps
		# n_epochs=10, # Number of policy updates per iteration of n_steps
		# gamma=0.99,
		# gae_lambda=0.95,
		# clip_range=0.2,
		# vf_coef=1,
		# ent_coef=0.01)

	# Load learning data
	model = PPO.load("PPO_results_3.zip")
	model.set_env(env)
	print("model loaded")


	# # Training
	# print("début de l'apprentissage")
	# model.learn(total_timesteps=100000)
	# t1 = time.time()
	# print("fin de l'apprentissage après " + str(t1-t0) + "secondes")
	# print("nombre de collisions : " +str(env.numero_crash))
	# print("nombre de problèmes de communication avec le lidar : " +str(env.nb_pb_lidar))
	# print("nombre de problèmes d'acquisition lidar : " +str(env.nb_pb_acqui_lidar))
	# print("nombre de démarrage du lidar : " +str(env.nb_demarrage_lidar))

	
	model.save("PPO_results_4") # Save learning data

	# Demo of the results
	
	obs = env.reset()
	print("Demo of the results.")
	c = 0
	for _ in range(10000):
		# Play the demo
		action, _ = model.predict(obs, deterministic=True)
		obs, reward, done, _ = env.step(action)
		c += reward
		if done:
			obs = env.reset()
	print("Exiting.")
	print(c)
	

if __name__ == '__main__' :
	main()
