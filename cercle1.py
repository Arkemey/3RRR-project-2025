"""
Robot 3RRR - Traçage sécurisé d'un cercle
+ Plateforme triangulaire visible
+ Vérification P au centre du triangle équilatéral C1-C2-C3
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from matplotlib.animation import FuncAnimation

# ============================================================================
# PARAMÈTRES DU ROBOT
# ============================================================================
Rb = 0.100      # Rayon du cercle des bases (100 mm)
L1 = 0.100      # Longueur bras actif Ai->Bi (100 mm)
L2 = 0.100      # Longueur bras passif Bi->Ci (100 mm)
R = 0.050       # Rayon plateforme mobile (50 mm)
SAFETY = 0.01   # Distance sécurité (10 mm)

param = [Rb, L1, L2, R]

print("="*70)
print("ROBOT 3RRR - Tracage securise d'un cercle")
print("="*70)

# ============================================================================
# CINÉMATIQUE INVERSE
# ============================================================================
def ikm(param, x, y, alpha, config='elbow_up'): #Renvoie les 3 angles moteurs ; la poistion des points A (la base)
#La position des points C (effecteur)
    """MGI: Calcule les angles moteurs"""
    Rb, L1, L2, R = param
    
    i = np.array([1, 2, 3])
    angles_base = 2*i*np.pi/3 + np.pi/2 #pi/2=120deg et pi/3 =90deg
    #Positions des points A
    xA = Rb * np.cos(angles_base)
    yA = Rb * np.sin(angles_base)
    #Positions des points C
    angles_platform = alpha + 2*i*np.pi/3 - np.pi/2 
    xC_rel = R * np.cos(angles_platform)
    yC_rel = R * np.sin(angles_platform)
    xC = x + xC_rel
    yC = y + yC_rel
    
    alpha_motors = np.zeros(3)
    #distance AiCi
    for i in range(3):
        dx = xC[i] - xA[i]
        dy = yC[i] - yA[i]
        d = np.sqrt(dx**2 + dy**2)
        #Vérification : si la distance peut etre atteinte par le bras
        if d > (L1 + L2) or d < abs(L1 - L2):
            return None
        #angle global du vecteur AC
        gamma = np.arctan2(dy, dx)
        #loi des cosinus pour l'articulation du bras
        cos_angle = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
        cos_angle = np.clip(cos_angle, -1, 1)
        angle_offset = np.arccos(cos_angle)
        
        if config == 'elbow_up':
            alpha_motors[i] = gamma + angle_offset
        else: #down
            alpha_motors[i] = gamma - angle_offset
    #résultats
    return alpha_motors, xA, yA, xC, yC

def get_B_positions(param, alpha_motors):
    """Calcule les positions des points Bi"""
    Rb, L1, L2, R = param
    #De nouveau Ai  
    i = np.array([1, 2, 3])
    angles_base = 2*i*np.pi/3 + np.pi/2
    xA = Rb * np.cos(angles_base)
    yA = Rb * np.sin(angles_base)
    #calcul des Bi : on avance juste de L1 par rapport à la direction de l'angle moteur alphai
    xB = xA + L1 * np.cos(alpha_motors)
    yB = yA + L1 * np.sin(alpha_motors)
    #résultats
    return xB, yB, xA, yA

# ============================================================================
# VÉRIFICATION GÉOMÉTRIQUE DE LA PLATEFORME
# ============================================================================
def verify_platform_geometry(x, y, xC, yC, R, tolerance=0.001):
    """
    Vérifie que:
    1. P est au centre du triangle C1-C2-C3
    2. Les Ci forment un triangle équilatéral de rayon R
    
    Returns: (is_valid, error_message)
    """
    # 1. Vérifier que P est au centre du triangle
    center_x = (xC[0] + xC[1] + xC[2]) / 3
    center_y = (yC[0] + yC[1] + yC[2]) / 3
    
    error_center = np.sqrt((center_x - x)**2 + (center_y - y)**2)
    
    if error_center > tolerance:
        return False, f"P pas au centre (erreur: {error_center*1000:.2f}mm)"
    
    # 2. Vérifier que les Ci sont à distance R de P
    for i in range(3):
        dist_PC = np.sqrt((xC[i] - x)**2 + (yC[i] - y)**2)
        error_R = abs(dist_PC - R)
        
        if error_R > tolerance:
            return False, f"C{i+1} pas a distance R (erreur: {error_R*1000:.2f}mm)"
    
    # 3. Vérifier triangle équilatéral
    dist_C1C2 = np.sqrt((xC[1] - xC[0])**2 + (yC[1] - yC[0])**2)
    dist_C2C3 = np.sqrt((xC[2] - xC[1])**2 + (yC[2] - yC[1])**2)
    dist_C3C1 = np.sqrt((xC[0] - xC[2])**2 + (yC[0] - yC[2])**2)
    
    mean_dist = (dist_C1C2 + dist_C2C3 + dist_C3C1) / 3
    max_error = max(abs(dist_C1C2 - mean_dist), 
                    abs(dist_C2C3 - mean_dist), 
                    abs(dist_C3C1 - mean_dist))
    
    if max_error > tolerance:
        return False, f"Triangle pas equilateral (erreur: {max_error*1000:.2f}mm)"
    
    return True, "ok"

# ============================================================================
# VÉRIFICATIONS DE SÉCURITÉ
# ============================================================================
def check_collision_silent(xB, yB, Rb, safety):
    """Vérifie collision"""
    for i in range(3):
        dist = np.sqrt(xB[i]**2 + yB[i]**2)
        if dist > (Rb - safety):
            return True
    return False

def check_singularity_serie_silent(xA, yA, xB, yB, xC, yC, threshold=10):
    """Vérifie singularité série"""
    for i in range(3):
        vec_AB = np.array([xB[i] - xA[i], yB[i] - yA[i]])
        vec_BC = np.array([xC[i] - xB[i], yC[i] - yB[i]])
        
        norm_AB = np.linalg.norm(vec_AB)
        norm_BC = np.linalg.norm(vec_BC)
        
        if norm_AB > 1e-6 and norm_BC > 1e-6:
            cos_angle = np.dot(vec_AB, vec_BC) / (norm_AB * norm_BC)
            cos_angle = np.clip(cos_angle, -1, 1)
            angle_deg = np.degrees(np.arccos(cos_angle))
            
            if angle_deg < threshold or angle_deg > (180 - threshold):
                return True
    return False

def check_singularity_parallele_silent(xA, yA, xB, yB):
    """Vérifie singularité parallèle"""
    vectors = []
    for i in range(3):
        dx = xB[i] - xA[i]
        dy = yB[i] - yA[i]
        norm = np.sqrt(dx**2 + dy**2)
        if norm > 0:
            vectors.append((dx/norm, dy/norm))
    
    cross_12 = abs(vectors[0][0]*vectors[1][1] - vectors[0][1]*vectors[1][0])
    cross_23 = abs(vectors[1][0]*vectors[2][1] - vectors[1][1]*vectors[2][0])
    cross_31 = abs(vectors[2][0]*vectors[0][1] - vectors[2][1]*vectors[0][0])
    
    if cross_12 < 0.05 and cross_23 < 0.05 and cross_31 < 0.05:
        return True
    
    def line_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
        denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(denom) < 1e-10:
            return None
        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom
        return (x1 + t*(x2-x1), y1 + t*(y2-y1))
    
    int_12 = line_intersection(xA[0], yA[0], xB[0], yB[0], xA[1], yA[1], xB[1], yB[1])
    int_23 = line_intersection(xA[1], yA[1], xB[1], yB[1], xA[2], yA[2], xB[2], yB[2])
    
    if int_12 and int_23:
        dist = np.sqrt((int_12[0]-int_23[0])**2 + (int_12[1]-int_23[1])**2)
        if dist < 0.02:
            return True
    
    return False

def is_point_safe(param, x, y, alpha):
    """
    Vérifie si un point est sûr
    AVEC vérification que P reste au centre du triangle
    """
    result = ikm(param, x, y, alpha, config='elbow_up')
    if result is None:
        return False, "hors_workspace", None
    
    alpha_motors, xA, yA, xC, yC = result
    xB, yB, xA, yA = get_B_positions(param, alpha_motors)
    
    # NOUVEAU: Vérifier géométrie de la plateforme
    is_valid, error_msg = verify_platform_geometry(x, y, xC, yC, R)
    if not is_valid:
        return False, f"geometrie_plateforme_{error_msg}", None
    
    if check_collision_silent(xB, yB, Rb, SAFETY):
        return False, "collision", None
    
    if check_singularity_serie_silent(xA, yA, xB, yB, xC, yC):
        return False, "singularite_serie", None
    
    if check_singularity_parallele_silent(xA, yA, xB, yB):
        return False, "singularite_parallele", None
    
    data = {
        'alpha_motors': alpha_motors,
        'xA': xA, 'yA': yA,
        'xB': xB, 'yB': yB,
        'xC': xC, 'yC': yC
    }
    
    return True, "ok", data

# ============================================================================
# GÉNÉRATION CERCLE
# ============================================================================
def generate_safe_circle(param, center, radius, n_points=100, alpha=0, max_attempts=5):
    """Génère un cercle sûr"""
    cx, cy = center
    current_radius = radius
    
    for attempt in range(max_attempts):
        print(f"\nTentative {attempt+1}/{max_attempts}: rayon={current_radius*1000:.1f}mm")
        
        angles = np.linspace(0, 2*np.pi, n_points, endpoint=False)
        path = []
        reasons_count = {}
        
        for angle in angles:
            x = cx + current_radius * np.cos(angle)
            y = cy + current_radius * np.sin(angle)
            
            safe, reason, data = is_point_safe(param, x, y, alpha)
            
            if safe:
                path.append({'x': x, 'y': y, 'alpha': alpha, 'data': data})
            else:
                reasons_count[reason] = reasons_count.get(reason, 0) + 1
        
        success_rate = len(path) / n_points * 100
        print(f"  Points surs: {len(path)}/{n_points} ({success_rate:.1f}%)")
        
        if reasons_count:
            print(f"  Echecs: {reasons_count}")
        
        if len(path) == n_points:
            print(f"  [OK] Cercle complet!")
            return path, True
        
        if attempt < max_attempts - 1:
            current_radius *= 0.85
    
    if path:
        print(f"\n[!] Cercle partiel: {len(path)} points")
        return path, False
    
    return [], False

# ============================================================================
# ANIMATION
# ============================================================================
def animate_circle(param, path):
    """Animation avec plateforme triangulaire visible"""
    if not path:
        return
    
    print(f"\nAnimation: {len(path)} points")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # AX1: Robot
    circle_safe = Circle((0, 0), Rb - SAFETY, fill=True, 
                        facecolor='lightgreen', alpha=0.3, label='Zone sure')
    ax1.add_patch(circle_safe)
    
    circle_limit = Circle((0, 0), Rb, fill=False, 
                         edgecolor='black', linewidth=3, label='Limite cercle')
    ax1.add_patch(circle_limit)
    
    x_path = [p['x'] for p in path]
    y_path = [p['y'] for p in path]
    ax1.plot(x_path, y_path, 'b--', linewidth=1, alpha=0.3, label='Chemin prevu')
    
    trajectory, = ax1.plot([], [], 'r-', linewidth=3, label='Trace stylo')
    bases, = ax1.plot([], [], 'ks', markersize=12, label='Bases Ai')
    arms1 = [ax1.plot([], [], 'b-', linewidth=3)[0] for _ in range(3)]
    arms2 = [ax1.plot([], [], 'purple', linewidth=3, linestyle='--')[0] for _ in range(3)]
    points_B, = ax1.plot([], [], 'bo', markersize=10, label='Points Bi')
    points_C, = ax1.plot([], [], 'mo', markersize=8, label='Points Ci')
    
    # NOUVEAU: Plateforme triangulaire
    platform_patch = Polygon([[0,0], [0,0], [0,0]], 
                            fill=True, facecolor='yellow', 
                            edgecolor='orange', linewidth=2, 
                            alpha=0.6, label='Plateforme')
    ax1.add_patch(platform_patch)
    
    stylo, = ax1.plot([], [], 'ro', markersize=14, label='Stylo (P)')
    
    ax1.set_xlim([-0.25, 0.25])
    ax1.set_ylim([-0.25, 0.25])
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9, loc='upper right')
    ax1.set_title('Robot 3RRR', fontsize=14, fontweight='bold')
    ax1.set_xlabel('x [m]', fontsize=11)
    ax1.set_ylabel('y [m]', fontsize=11)
    
    # AX2: Angles
    ax2.set_xlabel('Point', fontsize=11)
    ax2.set_ylabel('Angle [deg]', fontsize=11)
    ax2.set_title('Angles moteurs', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    alpha1_line, = ax2.plot([], [], 'b-', linewidth=2, label='alpha1')
    alpha2_line, = ax2.plot([], [], 'r-', linewidth=2, label='alpha2')
    alpha3_line, = ax2.plot([], [], 'g-', linewidth=2, label='alpha3')
    ax2.legend()
    
    traj_x, traj_y = [], []
    alphas = [[], [], []]
    
    def update(frame):
        if frame >= len(path):
            return
        
        point = path[frame]
        data = point['data']
        
        traj_x.append(point['x'])
        traj_y.append(point['y'])
        trajectory.set_data(traj_x, traj_y)
        
        xA, yA = data['xA'], data['yA']
        xB, yB = data['xB'], data['yB']
        xC, yC = data['xC'], data['yC']
        
        bases.set_data(xA, yA)
        points_B.set_data(xB, yB)
        points_C.set_data(xC, yC)
        stylo.set_data([point['x']], [point['y']])
        
        # Bras
        for i in range(3):
            arms1[i].set_data([xA[i], xB[i]], [yA[i], yB[i]])
            arms2[i].set_data([xB[i], xC[i]], [yB[i], yC[i]])
        
        # NOUVEAU: Mettre à jour la plateforme triangulaire
        platform_vertices = list(zip(xC, yC))
        platform_patch.set_xy(platform_vertices)
        
        # Angles
        for i in range(3):
            alphas[i].append(np.degrees(data['alpha_motors'][i]))
        
        pts = list(range(len(alphas[0])))
        alpha1_line.set_data(pts, alphas[0])
        alpha2_line.set_data(pts, alphas[1])
        alpha3_line.set_data(pts, alphas[2])
        
        ax2.set_xlim(0, len(path))
        ax2.set_ylim(-180, 180)
        
        ax1.set_title(f'Robot 3RRR - Point {frame+1}/{len(path)}', fontsize=14, fontweight='bold')
    
    anim = FuncAnimation(fig, update, frames=len(path), 
                        interval=50, repeat=True)
    
    plt.tight_layout()
    plt.show()

# ============================================================================
# MAIN
# ============================================================================
def main():
    print("\n" + "="*70)
    print("GENERATION CERCLE")
    print("="*70)
    
    cx = 0.0        # Centre X en mètres
    cy = 0.0        # Centre Y en mètres
    radius = 0.035  # Rayon en mètres (35 mm)
    n_points = 80   # Nombre de points
    
    print(f"Centre: ({cx*1000:.0f}, {cy*1000:.0f}) mm")
    print(f"Rayon: {radius*1000:.0f} mm")
    print(f"Points: {n_points}")
    
    path, success = generate_safe_circle(param, (cx, cy), radius, n_points)
    
    if not path:
        print("\n[X] Echec generation")
        return
    
    print(f"\n[OK] {len(path)} points generes")
    print("[!] P reste au centre du triangle C1-C2-C3 a chaque instant")
    
    animate_circle(param, path)

if __name__ == "__main__":
    main()