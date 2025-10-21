import time
from dynamixel_sdk import *
import math
import numpy as np
from matplotlib import pyplot as plt
from math import sin, cos
from numpy.linalg import inv

# Constantes pour la communication et les moteurs
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_MOVING_SPEED = 32
PROTOCOL_VERSION = 1.0
BAUDRATE = 2000000
#DEVICENAME = '/dev/ttyDXL'
DEVICENAME = '/dev/ttyDXL'
#DEVICENAME = 'COM5'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_MINIMUM_POSITION_VALUE = 0
ANGLE_MAXIMUM = 180
SAMPLING_INTERVAL = 0.30
DXL_IDs = [1, 2, 3]  # IDs des moteurs
MIDDLE_POSITION = DXL_MAXIMUM_POSITION_VALUE / 2
VITESSE_MAX = 100 # vitesse max des mouvements



# Gestionnaires de communication
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def set_moving_speed(dxl_id, speed):
    """Définit la vitesse du moteur."""
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_MOVING_SPEED, speed)
    if dxl_comm_result != COMM_SUCCESS:
        raise Exception(f"Erreur de définition de la vitesse pour le moteur {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error:
        raise Exception(f"Erreur de paquet pour le moteur {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")

def angle_to_position(angle):
    """Convertit un angle en radian en une position-incrément moteur."""
    #return int((angle / ANGLE_MAXIMUM) * (MIDDLE_POSITION) + MIDDLE_POSITION)
    return(int(angle*4096./(2*np.pi)+1024))


def position_to_angle(position):
    """Convertit une position moteur en un angle."""
    return ((position - MIDDLE_POSITION) / MIDDLE_POSITION) * ANGLE_MAXIMUM

def initialize_port():
    """Initialise la communication avec le port série."""
    if not portHandler.openPort():
        raise Exception("Impossible d'ouvrir le port série.")
    if not portHandler.setBaudRate(BAUDRATE):
        raise Exception("Impossible de configurer le baudrate.")

def enable_torque(dxl_id):
    """Active le couple du moteur spécifié."""
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        raise Exception(f"Erreur d'activation du couple pour le moteur {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error:
        raise Exception(f"Erreur de paquet pour le moteur {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")

def disable_torque(dxl_id):
    """Désactive le couple du moteur spécifié."""
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

def read_present_position(dxl_id):
    """Lit la position actuelle du moteur."""
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        raise Exception(f"Erreur de lecture de la position actuelle pour le moteur {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error:
        raise Exception(f"Erreur de paquet pour le moteur {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")
    return dxl_present_position

def move_to_positions(target_angles_list):
    """Déplace les moteurs selon les angles spécifiés."""
    max_steps = max(len(angles) for angles in target_angles_list)

    for step in range(max_steps):
        for i, dxl_id in enumerate(DXL_IDs):
            if step < len(target_angles_list[i]):
                target_angle = target_angles_list[i][step]

                # Vérification que l'angle est dans la plage admissible
                """if target_angle < -ANGLE_MAXIMUM:
                    target_angle = -ANGLE_MAXIMUM
                elif target_angle > ANGLE_MAXIMUM:
                    target_angle = ANGLE_MAXIMUM
                """

                # Convertir l'angle cible en position moteur
                target_position = angle_to_position(target_angle)
                #print(step,i,target_position)

                # Envoyer la commande de position au moteur
                dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
                    portHandler, dxl_id, ADDR_MX_GOAL_POSITION, target_position
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Erreur d'envoi de position pour le moteur {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error:
                    print(f"Erreur de paquet pour le moteur {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")

        # Attendre le temps d'échantillonnage
        time.sleep(SAMPLING_INTERVAL)



def trefle(theta):    
    a=0.04 # rayon max
    rho=a*((np.sin(2*theta))**2+(np.sin(4*theta))**2/2.)
    return rho

def MGI(x,y,theta):

    L1=0.100        # long segment 1
    L2=0.100        # long segment 2
    Rb=0.1322594    # Rayon base
    Re=0.070        # Rayon effecteur

    # TH de l'effecteur
    THEff=np.array([ [np.cos(theta), -np.sin(theta), x ], 
                     [np.sin(theta),  np.cos(theta), y ],
                     [0, 0, 1.] ])

    # angle R_i par rapport R_0
    ang1=np.array( [0., 2*np.pi/3., 4*np.pi/3.])

    # angle des positions O_i et E_i 
    ang2=np.array( [-np.pi/2., np.pi/6., 5*np.pi/6.])

    qa=np.array( [0., 0., 0.] )

    for i in range(3):

        # TH de R_i par rapport à R_0
        TH=np.array( [  [np.cos(ang1[i]), -np.sin(ang1[i]), Rb*np.cos(ang2[i]) ], 
                        [np.sin(ang1[i]),  np.cos(ang1[i]), Rb*np.sin(ang2[i]) ],
                        [0, 0, 1] ])

        # Position des points E_i dans R_E
        PEi_E=np.array( [ [Re*np.cos(ang2[i])], [Re*np.sin(ang2[i])], [1] ] )

        # Position des trois points E_i de l'effecteur dans R_0
        PEi_0=np.dot(THEff,PEi_E)

        # Position des points effecteur E_i dans les repères dans robots R_i
        PEi_i=np.dot(np.linalg.inv(TH),PEi_0)

        # MGI 2R plan : position points extremes Ei dans le repère du Ri du 2R plan
        posx=PEi_i[0,0] 
        posy=PEi_i[1,0]

        aux=(posx**2+posy**2-L1**2-L2**2)/(2*L1*L2)

        if abs(aux) < 1:
            beta=np.arccos(aux) # angle du coude : changer le signe pour la solution coude en haut
        else :
            beta=0 
            print("problème d atteignabilite")

        #print(beta)

        alpha=np.arctan2(posy,posx)-np.arctan2(L2*np.sin(beta),L1+L2*np.cos(beta))

        qa[i]=alpha # angles d'épaule - liaisons actives 

    return qa

def main():
    """Programme principal."""

    

    Npoints = 500
    target_angles_list=np.zeros((3,Npoints+1))

    xi=-0.0
    yi=0.0
    thetai=-0*np.pi/6

    #qai=MGI(xi,yi,thetai)

    #print(qai)

    xf=0.05
    yf=0.0
    thetaf=0*np.pi/2

    for i in range(Npoints+1):
        
        """
        # ligne droite
        x=xi+(xf-xi)*i/Npoints
        y=yi+(yf-yi)*i/Npoints
        theta=thetai+(thetaf-thetai)*i/Npoints

        qa=MGI(x,y,theta)

        target_angles_list[0,i]=(qa[0])
        target_angles_list[1,i]=(qa[1])   
        target_angles_list[2,i]=(qa[2])
        
        """     
        #trefle
        teta=2*np.pi*i/Npoints
        rho=trefle(teta)

        x=rho*np.cos(teta)
        y=rho*np.sin(teta)
           
        qa=MGI(x,y,0.)
                        
        target_angles_list[0,i]=(qa[0])
        target_angles_list[1,i]=(qa[1])           
        target_angles_list[2,i]=(qa[2])

    """target_angles_list[0,0]=(0)
    target_angles_list[1,0]=(0)
    target_angles_list[2,0]=(0)

    target_angles_list[0,1]=(np.pi/2)
    target_angles_list[1,1]=(0*np.pi/2)
    target_angles_list[2,1]=(-0*np.pi/6)
    """


    #print(target_angles_list)

    try:
        initialize_port()
        for dxl_id in DXL_IDs:
            enable_torque(dxl_id)
            set_moving_speed(dxl_id, VITESSE_MAX)

        #Initialisation
        move_to_positions([[0.31655754], [0.31655754], [0.31655754]] )
        time.sleep(5)

        #move_to_positions([ [qai[0]], [qai[1]], [qai[2]] ] )
        #time.sleep(5)

        move_to_positions(target_angles_list)  

        time.sleep(10)  
                
    except Exception as e:
        print(f"Erreur: {e}")
    
    finally:
        for dxl_id in DXL_IDs:
            disable_torque(dxl_id)
        portHandler.closePort()

if __name__ == "__main__":
    main()
