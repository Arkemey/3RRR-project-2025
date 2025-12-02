import time
import numpy as np
from dynamixel_sdk import *

ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
PROTOCOL_VERSION = 1.0
h = 0.07 # taille coté triangle equi 7cm
L = 0.1 # longueur des bras 10cm
R = 0.275/2  # rayon du cercle des servomoteurs
angles_moteurs = np.radians([0, 120, 240])
Ox = R*np.cos(angles_moteurs) #les trois points contenant les servomoteurs
Oy = R*np.sin(angles_moteurs)

DEVICE_NAME = "COM5"
BAUDRATE = 1000000

DXL_IDS = [1,2,3]

portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def degres_to_moteur(degres): 
    return int((degres / 360) *4096)

def enable_torque(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 1)

def disable_torque(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0)

def set_speed(dxl_id, speed):
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_MOVING_SPEED, speed)

def move_motor(dxl_id, tick):
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, tick) #tick de 0 à 4096

def initialisation(angle_pos):
    tick = degres_to_moteur(angle_pos)

    for dxl_id in DXL_IDS:
        enable_torque(dxl_id)
        set_speed(dxl_id, 80) #vitesse arbitraire, 80 c'est pas trop rapide ne risque pas d'endommager le materiel.
        move_motor(dxl_id, tick) #on met donc tout les motors à un angle initial, on calculera pour mettre G au centre du cercle

    time.sleep(5) #on attend 5s avant de commencer la suite du code.

def r_carre(x,y):
    return (x**2+y**2)

def inverse_kinematics(x,y):
    m = h / np.sqrt(3) 
    xC = [x+m, x-m/2, x-m/2] # positions des points Ci selon G(x,y)
    yC = [y, y +(m*np.sqrt(3))/2, y - m*np.sqrt(3)/2]
    angle_beta = [0,0,0]
    angle_alpha =[0,0,0]
    i = 0 
    while(i < 3): #désormais on fait inverse kinematics sur chacun des trois points pour avoir nos 3 angles moteurs.
        dx = xC[i] - Ox[i]
        dy = yC[i] - Oy[i]
        r_2 = r_carre(dx,dy)
        cos_beta = (r_2 - 2*L**2)/ (2*L**2)
        
        # Vérification domaine de validité pour arccos (évite les crashs) // GEMINI, à verifier. 
        val_acos = (r_2 - 2*L**2)/ (2*L**2)
        if val_acos > 1.0 or val_acos < -1.0:
            print(f"ERREUR: Point hors d'atteinte pour le bras {i}")
            return None, None, None # Indique une erreur
        
        angle_beta[i] = np.arccos(cos_beta)

        angle_alpha[i] = np.arctan2(dy,dx) - np.arctan2(L*np.sin(angle_beta[i]),L+ L*np.cos(angle_beta[i]))
        print("angle alpha :  " , angle_alpha[i] )
        print("angle beta :  " , angle_beta[i] )
        print("xC, yC", xC[i], yC[i])
        i = i + 1
    return np.degrees(angle_alpha)

def positionsB(alpha):
    Bx = []
    By = []
    i = 0
    while(i < 3):
        Bxi = Ox[i] + L*np.cos(alpha[i]) 
        Byi = Oy[i] + L*np.sin(alpha[i])
        Bx.append(Bxi)
        By.append(Byi)
    return Bx, By

def singularite(beta):
    if ((beta == 0)or(beta == np.pi)): #demo sur mon cahier tu le cestres
        return 0
    else : 
        return 1

def mouvement_vers(x,y):
    angle_a = inverse_kinematics(x,y)
    if angle_a is None:
        print("erreur, point non atteignable")
        return False

    #tick_angleA = degres_to_moteur(angle_a)
    #i = 1 # vu que les ids vont de 1 à 3.
    for i in range(3):
        tick = degres_to_moteur(angle_a[i])
        move_motor(DXL_IDS[i], tick)
        #tick_angleA = degres_to_moteur(angle_a[i-1])
        #move_motor(DXL_IDS[i],int(tick_angleA[i]))
        #i = i + 1
    return ("Mouvement vers (%f,%f)",x,y)


def main(): 
    if not portHandler.openPort():
        print("ERROR: Impossible d'ouvrir le port série.")
        quit()

    if not portHandler.setBaudRate(BAUDRATE):
        print("ERROR: Impossible de régler le baudrate.")
        quit()

    print("lancement du code")
    initialisation(110)
    print("parpitié)")
    # i = 0
    # trace_x, trace_y = [], []
    # target_points = []
    # while(i < 125):
    #     target_x = 0.00 + 0.02 * np.cos(i * 0.05) # Réduire le rayon à 2cm
    #     target_y = 0.00 + 0.02 * np.sin(i * 0.05)
    #     target_points.append((target_x, target_y))
    #     i += 1
    #     i += 1
    
    # initialisation(110.0) #valeur des angles pour etre au centre (déduis grace à dynamixel)

    # for x, y in target_points : 
    #     mouvement_vers(x,y)
    #     print("pos :", x,y)

    for dxl in DXL_IDS:
        disable_torque(dxl)

    portHandler.closePort()
    print("fin du code") 

main()