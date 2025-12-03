import time
import numpy as np
from dynamixel_sdk import *
from pynput import keyboard
import threading

ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
VITESSE = 40
PROTOCOL_VERSION = 1.0
h = 0.07 # taille coté triangle equi 7cm
L = 0.1 # longueur des bras 10cm
R = 0.275/2  # rayon du cercle des servomoteurs
angles_moteurs = np.radians([-83, 37, 157])
Ox = R*np.cos(angles_moteurs) #les trois points contenant l es servomoteurs
Oy = R*np.sin(angles_moteurs)
 
DEVICE_NAME = "/dev/ttyUSB0" 
BAUDRATE = 1000000

DXL_IDS = [1,2,3]
OFFSET = [-30, -30, -30]  #59.3 * 
 
portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def degres_to_moteur(degres): 
    return int((degres / 360) *4096)

stop_program = False

def on_press(key):
    global stop_program
    if key == keyboard.Key.space:
        stop_program = True
        print("\n STOOOOOOOOOOP !\n")
        return False  # Arrête le listener

def start_keyboard_listener():
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

def enable_torque(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 1)

def disable_torque(dxl_id):
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0)

def set_speed(dxl_id, speed):
    packetHandler.write2ByteTxRx(portHandler, dxl_id, VITESSE, speed)

def move_motor(dxl_id, tick):
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, tick) #tick de 0 à 4096

def initialisation(angle_pos):
    tick = degres_to_moteur(angle_pos)

    for dxl_id in DXL_IDS:
        enable_torque(dxl_id)
        set_speed(dxl_id, VITESSE) #vitesse arbitraire, 80 c'est pas trop rapide ne risque pas d'endommager le materiel.
        move_motor(dxl_id, tick) #on met donc tout les motors à un angle initial, on calculera pour mettre G au centre du cercle

    time.sleep(5) #on attend 5s avant de commencer la suite du code.

def r_carre(x,y):
    return (x**2+y**2)

def inverse_kinematics(x,y):
    m = h / np.sqrt(3) 
    xC = [x+m, x-m/2, x-m/2] # positions des points Ci selon G(x,y)
    yC = [y, y +(m*np.sqrt(3))/2, y - m*np.sqrt(3)/2]
    #xC = x + m*np.cos(angles_moteurs)
    #yC = y + m*np.sin(angles_moteurs)   
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
        print("POUR LE SERVOMOTEUR NUMERO :", i+1)
        print("angle alpha :  " , np.degrees(angle_alpha[i])+OFFSET[i])
        print("angle beta :  " , np.degrees(angle_beta[i])+OFFSET[i])
        print("xC, yC", "%.2f" % xC[i], "%.2f" % yC[i])
        i = i + 1
    return np.degrees(angle_alpha), np.degrees(angle_beta)

def positionsB(alpha):
    Bx = []
    By = []
    i = 0
    while(i < 3):
        Bxi = Ox[i] + L*np.cos(alpha[i]) 
        Byi = Oy[i] + L*np.sin(alpha[i])
        Bx.append(Bxi)
        By.append(Byi)
        i += 1
    return Bx, By

def singularite_serie(beta): #singularité série
    i = 0
    while(i < 3):  
        if ((beta[i] < 2)or((beta[i] > 178)and(beta[i])<182)): #demo sur mon cahier tu le cestres
            print("SINGULARITÉ DETECTÉ <-------------- !!!!!!!!!!")
            return 0
        i += 1
    return 1

def mouvement_vers(x,y):
    angle_a, _ = inverse_kinematics(x,y)
    _ , angle_b = inverse_kinematics(x,y)
    if angle_a is None:
        print("erreur, pas d'angle alpha")
        return False
    
    if angle_b is None: 
        print("erreur, pas d'angle beta")
        return False

    if (singularite_serie(angle_b)== 0) :
        return 0
     
    for i in range(3):
        tick = degres_to_moteur(angle_a[i]+OFFSET[i]) 
        move_motor(DXL_IDS[i], tick)
    return ("Mouvement vers (%f,%f)",x,y)


def main(): 
    start_keyboard_listener()
    if not portHandler.openPort():
        print("ERROR: Impossible d'ouvrir le port série.")
        quit()

    if not portHandler.setBaudRate(BAUDRATE):
        print("ERROR: Impossible de régler le baudrate.")
        quit()

    print("Lancement du code")
    initialisation(110)
    print("Initalisation terminé, début du traçage")
    i = 0

    target_points = []
    while(i < 125):
        # TRAJECTOIRE CERCLE : 
        # target_x = 0.00 + 0.01* np.cos(i * 0.01) # Réduire le rayon à 1cm
        # target_y = 0.00 + 0.01* np.sin(i * 0.01)
        #TRAJECTOIRE LIGNE :
        target_x = 0.01*i
        target_y = 0.0
        target_points.append((target_x, target_y))
        i += 1
        i += 1
    
    initialisation(110.0) #valeur des angles pour etre au centre (déduis grace à dynamixel)

    for x, y in target_points : 
        if stop_program :
            break

        mouvement_vers(x,y) 
        print("pos :", x,y)

    for dxl in DXL_IDS:
        disable_torque(dxl)

    portHandler.closePort()
    print("fin du code") 

main() 


