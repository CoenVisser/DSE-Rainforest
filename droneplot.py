import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# === PLATFORM DIMENSIONS ===
l_platform = 0.15     # length
w_platform = 0.062    # width
h_platform = 0.059    # height

# === ARM PARAMETERS ===
arms = [
    {
        "length": 0.163652041,
        "alpha": 44.9999255,
        "beta": -32.3215876,
        "color": "blue",
        "label": "Propeller"
    },
    {
        "length": 0.19829375324306342,
        "alpha": 0,
        "beta": -10.3063,
        "color": "orange",
        "label": "Init Bumper"
    },
    {
        "length": 0.2254,
        "alpha": 13.0318,
        "beta": 10.3063,
        "color": "green",
        "label": "Bumper"
    },
    {
        "length": 0.2322,
        "alpha": 19.1459,
        "beta": 10.0,
        "color": "red",
        "label": "Spine"
    }
]

# === 3D DIRECTION COMPUTATION ===
def arm_endpoints_3d(alpha_deg, beta_deg, length):
    alpha = np.deg2rad(alpha_deg)
    beta = np.deg2rad(beta_deg)
    x = length * np.cos(beta) * np.cos(alpha)
    y = length * np.cos(beta) * np.sin(alpha)
    z = length * np.sin(beta)
    return np.array([x, y, z])

# === CENTRAL BODY CORNERS ===
def platform_box_coords():
    dx, dy, dz = l_platform / 2, w_platform / 2, h_platform / 2
    corners = [
        [-dx, -dy, -dz], [dx, -dy, -dz], [dx, dy, -dz], [-dx, dy, -dz],
        [-dx, -dy, dz], [dx, -dy, dz], [dx, dy, dz], [-dx, dy, dz]
    ]
    return np.array(corners)

# === SETUP PLOTS ===
def plot_drone_2D():
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ['Top View (XY)', 'Front View (XZ)', 'Side View (YZ)']
    projections = [(0, 1), (0, 2), (1, 2)]

    for ax, (i, j), title in zip(axes, projections, titles):
        ax.set_title(title)
        ax.set_aspect('equal')
        ax.set_xlabel(['X','Y','Z'][i])
        ax.set_ylabel(['X','Y','Z'][j])
        
        # --- PLATFORM ---
        corners = platform_box_coords()
        faces = [(0,1,2,3), (4,5,6,7), (0,1,5,4), (2,3,7,6), (1,2,6,5), (3,0,4,7)]
        for face in faces:
            face_indices = np.array(face)
            coords = corners[face_indices][:, [i, j]]
            coords = np.vstack([coords, coords[0]])  # close loop
            ax.plot(coords[:, 0], coords[:, 1], 'k-', linewidth=1)

        # --- ARMS ---
        for arm in arms:
            for sign_x in [1, -1]:
                for sign_y in [1]:
                    dx, dy, dz = arm_endpoints_3d(arm["alpha"] * sign_x, arm["beta"] * sign_y, arm["length"])
                    ax.plot([0, dx], [0, [dy, dz][j == 2]], color=arm["color"], label=arm["label"])

        # --- COG INDICATOR ---
        ax.plot(0, 0, 'kx', markersize=8, label='CoG')
        ax.text(0, 0, '  CoG', verticalalignment='bottom', horizontalalignment='left', fontsize=10, color='black')

        # Only show unique legend labels once
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        ax.legend(unique.values(), unique.keys(), loc='upper right')
        ax.grid(True)

    plt.tight_layout()
    plt.show()

# === 3D DIRECTION COMPUTATION ===
def arm_endpoints_3d(alpha_deg, beta_deg, length):
    alpha = np.deg2rad(alpha_deg)
    beta = np.deg2rad(beta_deg)
    x = length * np.cos(beta) * np.cos(alpha)
    y = length * np.cos(beta) * np.sin(alpha)
    z = length * np.sin(beta)
    return np.array([x, y, z])

# === PLATFORM BOX CORNERS ===
def platform_box_coords():
    dx, dy, dz = l_platform / 2, w_platform / 2, h_platform / 2
    corners = [
        [-dx, -dy, -dz], [dx, -dy, -dz], [dx, dy, -dz], [-dx, dy, -dz],
        [-dx, -dy, dz], [dx, -dy, dz], [dx, dy, dz], [-dx, dy, dz]
    ]
    return np.array(corners)

# === DRAW 3D PLOT ===
def plot_drone_3D():
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D View of Drone Cross Section")

    # --- PLATFORM ---
    corners = platform_box_coords()
    faces = [
        [0,1,2,3], [4,5,6,7],  # bottom, top
        [0,1,5,4], [2,3,7,6],  # front, back
        [1,2,6,5], [3,0,4,7]   # right, left
    ]
    face_polys = [corners[face] for face in faces]
    ax.add_collection3d(Poly3DCollection(face_polys, facecolors='gray', linewidths=1, edgecolors='k', alpha=0.4))

    # --- ARMS ---
    for arm in arms:
        for sign_x in [1, -1]:
            for sign_y in [1, -1]:
                vec = arm_endpoints_3d(arm["alpha"] * sign_x, arm["beta"] * sign_y, arm["length"])
                ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=arm["color"], label=arm["label"])

    # --- COG MARKER ---
    ax.scatter(0, 0, 0, color='black', marker='x', s=50)
    ax.text(0, 0, 0, '  CoG', color='black')

    # --- AXIS SETTINGS ---
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    ax.set_box_aspect([1,1,0.7])  # more natural aspect ratio
    ax.view_init(elev=25, azim=135)

    # --- LEGEND (unique labels only) ---
    handles, labels = ax.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    ax.legend(unique.values(), unique.keys(), loc='upper right')

    plt.tight_layout()
    plt.show()