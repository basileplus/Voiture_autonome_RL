# Python imports
import random
import time
import sys
import struct

# Webots imports
from controller import Supervisor



# Global constants
PI = 3.141592653589793
RECEIVER_SAMPLING_PERIOD = 64	# In milliseconds

# /!\ Set the number of sparring partner cars
NB_SPARRINGPARTNER_CARS = 3

# Clipping function
def value_clip(x, low, up):
	return low if x < low else up if x > up else x

# Angle normalization function (Webots angle values range between -pi and pi)
def angle_clip(x):
	a = x % (2*PI)
	return a if a < PI else a - 2*PI

# Positions intiales aléatoires 
# (plusieurs positions initiales auxquelles s'ajoute un peu d'aléatoire)
# 	[[x_min, x_max], [y_min, y_max], rotation]
# Les angles correspondents au même sens de rotation pour les voitures


# Train track
starting_positions = [
 	[[ 0.00,  4.00], [ 5.10,  5.30],  0.00],
 	[[ 4.95,  5.15], [ 0.30,  4.00], -PI/2],
 	[[ 2.70,  4.50], [-1.25, -0.75],  PI  ],
 	[[ 3.20,  3.45], [ 2.00,  2.40],  PI/2],
 	[[ 1.90,  2.70], [ 3.10,  3.30],  PI  ],
 	[[ 0.00,  0.25], [-0.50,  1.50], -PI/2],
 	[[-2.20, -1.90], [-0.50,  2.30],  PI/2]
]

"""
# Test track
starting_positions = [
	[[ 0.60,  4.20], [ 5.40,  5.60],  0.00],
	[[ 5.40,  5.60], [-4.30,  4.30], -PI/2],
	[[ 1.40,  1.60], [-5.00, -4.30],  PI/2],
	[[ 3.40,  3.60], [-2.85, -1.45],  PI/2],
	[[ 3.40,  3.60], [ 2.20,  2.50],  PI/2],
	[[-0.60, -0.40], [ 0.30,  0.90], -PI/2],
	[[-3.20, -2.00], [-4.60, -4.40],  PI],
	[[-2.60, -2.40], [-0.80,  2.30],  PI/2],
]
"""

# Initialisation
supervisor = Supervisor()
basicTimeStep = int(supervisor.getBasicTimeStep())
print("basic time step : "+str(basicTimeStep) + "ms")

# Reset Receiver et emitter
ResetReceiver = supervisor.getDevice("ResetReceiver")
ResetReceiver.enable(RECEIVER_SAMPLING_PERIOD)
ResetReceiver.setChannel(1)
ResetEmitter = supervisor.getDevice("ResetEmitter")
ResetEmitter.setChannel(2)
packet_number = 0

# Lap and sector times emitter
LapEmitter = supervisor.getDevice("LapEmitter")
LapEmitter.setChannel(3)


# Initialize crossing-line times
line1_t, line2_t, line3_t = 0,0,0

# Recuperation des liens vers les noeuds voitures
tt_02 = supervisor.getFromDef("TT02_2023b_RL")
tt_02_translation = tt_02.getField("translation")
tt_02_rotation = tt_02.getField("rotation")
sparringpartner_car_nodes = [supervisor.getFromDef(f"sparringpartner_car_{i}") for i in range(NB_SPARRINGPARTNER_CARS)]
sparringpartner_car_translation_fields = [sparringpartner_car_nodes[i].getField("translation") for i in range(NB_SPARRINGPARTNER_CARS)]
sparringpartner_car_rotation_fields = [sparringpartner_car_nodes[i].getField("rotation") for i in range(NB_SPARRINGPARTNER_CARS)]

# Create a struct (Python object) for the TT-02 car node
tt_02.sector1_t = 0
tt_02.sector2_t = 0
tt_02.sector3_t = 0
tt_02.lap_time = 0

erreur_position = 0
# Main loop
while supervisor.step(basicTimeStep) != -1:
    # Si la ligne 1 est franchie
    if 4.5 < tt_02_translation.getSFVec3f()[0] < 5.8 and 1.95 < tt_02_translation.getSFVec3f()[1] < 2:
        # print("line 1 crossed")
        # print("DEBUG : s1_t = ", tt_02.sector1_t)
        # print("DEBUG : s2_t = ", tt_02.sector2_t)
        # print("DEBUG : s3_t = ", tt_02.sector3_t)
        # print("DEBUG : lap_time = ", tt_02.lap_time)
        if line1_t==0:
            line1_t = supervisor.getTime()
            if line3_t!=0:
                tt_02.sector3_t = line1_t -line3_t
                print("sector 3 time = ", tt_02.sector3_t)
        else :
            tt_02.lap_time = supervisor.getTime() - line1_t
            line1_t = supervisor.getTime()
            tt_02.sector3_t = line1_t -line3_t
            print("sector 3 time = ", tt_02.sector3_t)
            print("lap_time = ",tt_02.lap_time)
            
        # Send lap time and sector times
        message = struct.pack("ffff", tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t, tt_02.lap_time) # Pack lap time and sector times into a message
        print("super sends : ", message)
        LapEmitter.send(message)
        # A chaque tour on réinitialise les temps
        line2_t, line3_t = 0, 0
            
    # Si la ligne 2 est franchie
    if tt_02_translation.getSFVec3f()[0] < -1.48 and tt_02_translation.getSFVec3f()[0] > -2.76 and 1.25 < tt_02_translation.getSFVec3f()[1]<1.3:
        print("line 2 crossed")
        if line2_t==0 and line1_t!=0:
            line2_t = supervisor.getTime()
            tt_02.sector1_t = line2_t - line1_t

            # Send lap time and sector times
            message = struct.pack("ffff", tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t, tt_02.lap_time)
            print("super sends : ", message)
            LapEmitter.send(message)
            print("sector 1 time = ", tt_02.sector1_t)
        else :
            print("ligne 2 franchie avant ligne 1")
            
            
    # Si la ligne 3 est franchie
    if 2.7 < tt_02_translation.getSFVec3f()[0] < 4 and  2.15 < tt_02_translation.getSFVec3f()[1] < 2.2:
        print("line 3 crossed")
        if line3_t==0 and line2_t!=0 :
            line3_t = supervisor.getTime()
            tt_02.sector2_t = line3_t - line2_t

            # Send lap time and sector times
            message = struct.pack("ffff", tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t, tt_02.lap_time)
            print("super sends : ", message)
            LapEmitter.send(message)
            print("sector 2 time = ", tt_02.sector2_t)

        else:
            print("ligne 3 franchie avant ligne 2")
       

    # detection de positions incoherentes. Ne devrait pas servir...
    if (abs(tt_02_translation.getSFVec3f()[0]) > 30) or (abs(tt_02_translation.getSFVec3f()[1]) > 30) or (abs(tt_02_translation.getSFVec3f()[2]) > 10) or ((abs(tt_02_rotation.getSFVec3f()[3]) > PI/2) and ((abs(tt_02_rotation.getSFVec3f()[0]) > 0.5) or (abs(tt_02_rotation.getSFVec3f()[1]) > 0.5))):
        start_rot = [0.0, 0.0, 1.0, -PI/2]
        tt_02_rotation.setSFRotation(start_rot)
        tt_02.setVelocity([0,0,0,0,0,0])
        tt_02_translation.setSFVec3f([2.98, 0.34, 0.039]) #devant un mur
        erreur_position += 1
        supervisor.step(basicTimeStep)
        print("erreur position numero : ",erreur_position)
        
    # If reset signal : replace the cars
    if ResetReceiver.getQueueLength() > 0:
	# Get the data off the queue
        try :
            data = ResetReceiver.getString()
            print("super receive : ", data)
            ResetReceiver.nextPacket()
            print(data)
            # Choose driving direction
            direction = 1
    	# Select random starting points
            coordinates_idx = random.sample(range(len(starting_positions)), 1+NB_SPARRINGPARTNER_CARS)
    
    		# Replace TT-02 avec une position pseudo aleatoire
    		# print("replacement de la voiture violette")
            coords = starting_positions[coordinates_idx[0]]
            start_x = random.uniform(coords[0][0], coords[0][1])
            start_y = random.uniform(coords[1][0], coords[1][1])
            start_z = 0.039
    		
            angle = coords[2]
            start_angle = random.uniform(angle - PI/12, angle + PI/12)
            if direction:
                start_angle = start_angle + PI
                start_rot = [0.0, 0.0, 1.0, angle_clip(start_angle)]
                tt_02_rotation.setSFRotation(start_rot)		
                tt_02_translation.setSFVec3f([start_x, start_y, start_z])
                tt_02.setVelocity([0,0,0,0,0,0])
    		
                packet_number += 1
                
                
                # Replace sparring partner cars
                for i in range(NB_SPARRINGPARTNER_CARS):
                    sparringpartner_car_nodes[i].setVelocity([0,0,0,0,0,0])
                    coords = starting_positions[coordinates_idx[i + 1]]
                    start_x = random.uniform(coords[0][0], coords[0][1])
                    start_y = random.uniform(coords[1][0], coords[1][1])
                    start_z = 0.039
                    sparringpartner_car_translation_fields[i].setSFVec3f([start_x, start_y, start_z])
                    # Rotate sparringpartner cars
                    angle = coords[2]
                    start_angle = random.uniform(angle - PI/12, angle + PI/12)
                    if direction:
                        start_angle = start_angle + PI
                        start_rot = [0, 0, 1, angle_clip(start_angle)]
                        sparringpartner_car_rotation_fields[i].setSFRotation(start_rot)
                        sparringpartner_car_nodes[i].setVelocity([0,0,0,0,0,0])
                

                supervisor.step(basicTimeStep)
                message = "voiture replacee num : " + str(packet_number)
                print("super sends : ", message)
                ResetEmitter.send(message)
                # print("voiture replacee num : " + str(packet_number))
            
            # Reset every timing
            start_t = supervisor.getTime()
            line1_t, line2_t, line3_t = 0, 0, 0		
            tt_02.lap_time, tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t = 0, 0, 0, 0 # Reset lap time and sector times
            # Send lap time and sector times
            message = struct.pack("ffff", tt_02.sector1_t, tt_02.sector2_t, tt_02.sector3_t, tt_02.lap_time)
            print("super sends : ", message)
            LapEmitter.send(message)

        except :
            print("souci de réception")
	
