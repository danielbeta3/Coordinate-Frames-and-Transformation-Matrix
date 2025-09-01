import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from stl import mesh

# Matrices de rotacion y normalizar
def normalize(v, eps=1e-12): # normaliza los vectores, sirve para despues plotear
    n = np.linalg.norm(v)
    return v if n < eps else v / n

def rotz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], float)

def roty(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], float)

# Cálculo de α, β desde (u,v,w) y viento 
def alpha_beta_from_body(u, v, w, uw=0.0, vw=0.0, ww=0.0):
    """
    Calcula alpha (AoA) y beta (sideslip) en rad a partir de la velocidad
    relativa al viento en ejes BODY. Devuelve (alpha, beta, Vhat_b, V_air_b).
    """
    ur, vr, wr = u - uw, v - vw, w - ww # calcula el movimiento relativo al viento
    V_air_b = np.array([ur, vr, wr], float)
    Vmag    = np.linalg.norm(V_air_b) or 1e-12 #np.linalg saca la magnitud del vector
    Vhat_b  = V_air_b / Vmag

    alpha = np.arctan2(wr, ur)                         # [rad]
    beta  = np.arcsin(np.clip(vr / Vmag, -1.0, 1.0))   # [rad]
    return alpha, beta, Vhat_b, V_air_b

# Vectores
def build_vectors(Vhat_b, alpha, beta):
    """
    Devuelve un dict con:
      Vhat_b        : dirección de la velocidad en BODY
      V_alpha_b     : Vhat_b @ Rz(alpha)
      V_beta_b      : Vhat_b @ Rz(alpha) @ Ry(beta)
      V_rotY90_b    : Rotación +90° alrededor de Y aplicada a Vhat_b
      V_rotZneg_b   : Rotación -90° alrededor de Z aplicada a Vhat_b
    """
    V_alpha_b = normalize(Vhat_b @ rotz(alpha)) #normaliza el vector y lo rota 
    V_beta_b  = normalize(Vhat_b @ rotz(alpha) @ roty(beta)) #normaliza el vector y lo rota 

    RotY_90  = np.array([[0,0,1],[0,1,0],[-1,0,0]], float)  # +90° Y esto tener los vectores del cuerpo
    RotZ_m90 = np.array([[0,1,0],[-1,0,0],[0,0,1]], float)  # -90° Z
    V_rotY90_b  = normalize(RotY_90  @ Vhat_b)
    V_rotZneg_b = normalize(RotZ_m90 @ Vhat_b)

    return {
        "Vhat_b": Vhat_b,
        "V_alpha_b": V_alpha_b,
        "V_beta_b": V_beta_b,
        "V_rotY90_b": V_rotY90_b,
        "V_rotZneg_b": V_rotZneg_b,
    }

# Estructura de salida en el formato pedido
def aircraft_state(alpha, beta, u, v, w, p, q, r, phi, theta, psi):
    """
    Devuelve el diccionario EXACTO solicitado.
      - alpha, beta en rad (se convierten a deg aquí)
      - phi, theta, psi en rad (se dejan en rad)
    """
    alpha_deg = np.degrees(alpha)
    beta_deg  = np.degrees(beta)
    climb_deg = np.degrees(theta) - alpha_deg  # gamma [deg]

    state_values = {
        "angles": {
            "alpha": alpha_deg,     # Angle of attack [deg]
            "beta":  beta_deg,      # Sideslip angle [deg]
            "gamma": climb_deg      # climb angle [deg]
        },
        "velocities_body": np.array([u, v, w], float),   # [m/s]
        "angular_rates":   np.array([p, q, r], float),   # [rad/s]
        "attitude":        np.array([np.degrees(phi), np.degrees(theta), np.degrees(psi)], float)  # [rad]
    }
    return state_values

# Plot (opcional)
def plot_aircraft_with_vectors(stl_path, alpha, beta, vecs_body, show_aircraft=True):
    """
    Dibuja el STL (si show_aircraft=True) y los vectores.
    Alineación BODY->WORLD usando exactamente RotZ_beta @ RotY_alpha.
    """
    RotZ_beta  = rotz(beta)
    RotY_alpha = roty(alpha)
    R_align    = RotZ_beta @ RotY_alpha

    # Pasar vectores al plot
    def W(vb): return vb @ R_align.T
    Vhat_w     = W(vecs_body["Vhat_b"])
    Valpha_w   = W(vecs_body["V_alpha_b"])
    Vbeta_w    = W(vecs_body["V_beta_b"])
    VrotY90_w  = W(vecs_body["V_rotY90_b"])
    VrotZneg_w = W(vecs_body["V_rotZneg_b"])

    # Figura
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    cx = cy = cz = 0.0
    span = 2.0

    # aca solo agregamos una opción de mostrar o no el avión
    # para poder visualizar mejor los vectores
    if show_aircraft:
        #Carga el archivo .stl con numpy-stl. av.vectors trae los triángulos:
        # arreglo de forma (Nf, 3, 3) = Nf caras, cada una con 3 vértices, cada vértice con (x,y,z).
        av = mesh.Mesh.from_file(stl_path) # Esto convierte el archivo stl en una colección de puntos y aristas
        #Copia esos triángulos para no modificar el objeto original.
        faces = av.vectors.copy()
        # Aplana a una lista de vértices (Nf*3, 3), calcula el centroide (promedio de todos los puntos).
        #Traslada toda la geometría para que el centroide quede en el origen. Así el avión queda “centrado” en (0,0,0).
        centroid = faces.reshape(-1,3).mean(axis=0)
        faces -= centroid
        #np.ptp(..., axis=0) calcula el rango (máx − mín) de la malla en X, Y, Z.
        ext = np.ptp(faces.reshape(-1,3), axis=0)
        #Encuentra la mayor extensión y calcula un factor de escala S 
        # para que ese tamaño máximo pase a valer ≈ 2 unidades.
        S = 2.0 / (np.max(ext) if np.max(ext) > 0 else 1.0)
        faces *= S
        faces_w = faces @ R_align.T # .T es la transpuesta
        #Recalcula el centro (ya debería estar cerca de 0 por el centrado previo). 
        # Se usa como origen para dibujar flechas y textos.
        verts_w = faces_w.reshape(-1,3) 
        cx, cy, cz = verts_w.mean(axis=0)
        span = max(np.ptp(verts_w[:,0]), np.ptp(verts_w[:,1]), np.ptp(verts_w[:,2]), 1.0)
        #Crea una colección de polígonos 3D con los triángulos faces_w y la añade al Axes3D.
        ax.add_collection3d(Poly3DCollection(
            faces_w, facecolor="#2265d9", edgecolor='k', linewidths=0.3, alpha=0.95
        ))

    L = 0.45 * span
    def qv(vec, color, label):
        ax.quiver(cx, cy, cz, vec[0]*L, vec[1]*L, vec[2]*L,
                  color=color, linewidth=2, arrow_length_ratio=0.1)
        ax.text(cx + vec[0]*L, cy + vec[1]*L, cz + vec[2]*L, label, color=color)

    # Vectores
    qv(Vhat_w,     'm',     'V̂')
    qv(Valpha_w,   'lime',  'uvw@Rz(α)')
    qv(Vbeta_w,    'c',     'uvw@Rz(α)@Ry(β)')
    qv(VrotY90_w,  'k',     '+90° Y')
    qv(VrotZneg_w, 'orange','−90° Z')

    ax.set_xlim(cx-span/2, cx+span/2)
    ax.set_ylim(cy-span/2, cy+span/2)
    ax.set_zlim(cz-span/2, cz+span/2)
    ax.set_box_aspect([1,1,1])
    ax.set_title(f"α={np.degrees(alpha):.2f}°  β={np.degrees(beta):.2f}°")
    plt.show()
