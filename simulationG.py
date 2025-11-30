import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- CONSTANTES ET PARAMÈTRES DU ROBOT (Vos notations) ---
h = 0.08       # taille coté triangle equi 8cm
L = 0.1        # longueur des bras 10cm
R = 0.275/2       # rayon du cercle des servomoteurs
angles_moteurs = np.radians([0, 120, 240])
Ox = R * np.cos(angles_moteurs) 
Oy = R * np.sin(angles_moteurs)


def r_carre(x,y):
    return (x**2+y**2)

def inverse_kinematics(x,y):
    m = h / np.sqrt(3) 
    xC = [x+m, x-m/2, x-m/2] 
    yC = [y, y +(m*np.sqrt(3))/2, y - m*np.sqrt(3)/2]
    angle_beta = [0.0, 0.0, 0.0] 
    angle_alpha =[0.0, 0.0, 0.0]
    
    for i in range(3): 
        dx = xC[i] - Ox[i]
        dy = yC[i] - Oy[i]
        
        r_2 = r_carre(dx,dy) 
        val_acos = (r_2 - 2*L**2)/ (2*L**2)
        
        # Vérification domaine de validité pour arccos 
        if val_acos > 1.0 or val_acos < -1.0:
            return [], [], [] # Retourne 3 listes vides en cas d'erreur
        
        angle_beta[i] = np.arccos(val_acos)

        angle_alpha[i] = np.arctan2(dy,dx) - np.arctan2(L*np.sin(angle_beta[i]),L+ L*np.cos(angle_beta[i]))
        
    return np.array(angle_alpha), np.array(xC), np.array(yC)

def positionsB(alpha):
    Bx = Ox + L * np.cos(alpha) 
    By = Oy + L * np.sin(alpha)
    
    return Bx, By

# --- 3. SIMULATION ET ANIMATION MATPLOTLIB --- GEMINI :

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-0.4, 0.4)
ax.set_ylim(-0.4, 0.4)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Simulation Robot 3RRR Planar (Final)")
ax.scatter(Ox, Oy, color='blue', marker='o', label='Moteurs (O)') 

# Éléments graphiques
bras_lines = [ax.plot([], [], 'o-', lw=2, color='gray')[0] for _ in range(3)] 
plat_line, = ax.plot([], [], 'r-', lw=3, label='Plateforme (C)')
trace_line, = ax.plot([], [], 'g:', lw=1, alpha=0.5) 
text_info = ax.text(-0.35, 0.35, "", fontsize=10, verticalalignment='top')

trace_x, trace_y = [], []
target_points = []

# Création de la trajectoire 
i = 0
while(i < 125):
    target_x = 0.00 + 0.02 * np.cos(i * 0.05) # Réduire le rayon à 2cm
    target_y = 0.00 + 0.02 * np.sin(i * 0.05)
    target_points.append((target_x, target_y))
    i += 1
    i += 1

def init():
    for line in bras_lines:
        line.set_data([], [])
    plat_line.set_data([], [])
    trace_line.set_data([], [])
    text_info.set_text("")
    return bras_lines + [plat_line, trace_line, text_info]

def update(frame):
    if frame >= len(target_points):
        return bras_lines + [plat_line, trace_line, text_info]

    target_x, target_y = target_points[frame]
    
    # 1. Appel de l'IK
    angles_rad, xC, yC = inverse_kinematics(target_x, target_y)
    
    if not angles_rad.size: # Vérification si le tableau est vide (erreur)
        text_info.set_text(f"HORS D'ATTEINTE : ({target_x:.3f}, {target_y:.3f})")
        text_info.set_color('red')
        return bras_lines + [plat_line, trace_line, text_info]

    # 2. FK pour trouver les coudes B
    Bx, By = positionsB(angles_rad)

    # Mise à jour des dessins
    plat_x_draw = np.append(xC, xC[0]) # Fermer le triangle
    plat_y_draw = np.append(yC, yC[0])
    plat_line.set_data(plat_x_draw, plat_y_draw)
    
    for i in range(3):
        # Dessin du bras : O -> B -> C
        bras_lines[i].set_data([Ox[i], Bx[i], xC[i]], [Oy[i], By[i], yC[i]])

    # Trace du centre G
    trace_x.append(target_x)
    trace_y.append(target_y)
    trace_line.set_data(trace_x, trace_y)

    text_info.set_text(f"Pos G: ({target_x:.3f}, {target_y:.3f})\nAngles α (deg):\n{np.degrees(angles_rad).round(2)}")
    text_info.set_color('green')

    return bras_lines + [plat_line, trace_line, text_info]

ani = FuncAnimation(fig, update, frames=len(target_points), init_func=init, blit=True, interval=50)
plt.show()
