import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# === PLATFORM DIMENSIONS ===
l_platform = 0.15     # length
w_platform = 0.062    # width
h_platform = 0.059    # height

# === CENTRAL BODY CORNERS ===
def platform_box_coords():
    dx, dy, dz = l_platform / 2, w_platform / 2, h_platform / 2
    corners = [
        [-dx, -dy, -dz], [dx, -dy, -dz], [dx, dy, -dz], [-dx, dy, -dz],
        [-dx, -dy, dz], [dx, -dy, dz], [dx, dy, dz], [-dx, dy, dz]
    ]
    return np.array(corners)

def generate_attachments(point):
    base_start = np.array(point["attachment_point"])
    base_end = np.array(point["end_point"])
    vec = base_end - base_start

    arms = []

    def mirror(x_mirror=False, y_mirror=False):
        sign = np.array([
            -1 if x_mirror else 1,
            -1 if y_mirror else 1,
            1  # z remains unchanged
        ])
        new_start = sign * base_start
        new_end = new_start + sign * vec
        arms.append({
            "attachment_point": new_start,
            "end_point": new_end,
            "length": point["length"],
            "alpha": point["alpha"],
            "beta": point["beta"],
            "color": point["color"],
            "label": point["label"]
        })

    label = point["label"].lower()

    if label == "propeller":
        # 4: original, xz mirror, yz mirror, both
        mirror(False, False)
        mirror(True, False)
        mirror(False, True)
        mirror(True, True)

    elif label == "spine":
        # 2: on negative x side, mirrored over xz
        mirror(True, False)
        mirror(True, True)

    elif label == "foot":
        # 2: on positive x side, mirrored over xz
        mirror(False, False)
        mirror(False, True)

    elif label == "bumper":
        # Only one
        mirror(False, False)

    return arms

# === SETUP PLOTS ===
def plot_drone_2D(arms):
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ['Top View (XY)', 'Side View (XZ)', 'Front View (YZ)']
    projections = [(0, 1), (0, 2), (1, 2)]

    all_arms = []
    for arm in arms:
        all_arms.extend(generate_attachments(arm))

    # --- Compute all points (platform + arms) ---
    platform = platform_box_coords()
    all_points = [platform]
    for arm in all_arms:
        all_points.append(np.array([arm["attachment_point"], arm["end_point"]]))
    all_points = np.vstack(all_points)

    # --- Compute global max range across all axes ---
    min_vals = np.min(all_points, axis=0)
    max_vals = np.max(all_points, axis=0)

    # unified scale across x, y, z
    overall_min = np.min(min_vals)
    overall_max = np.max(max_vals)
    centre = (overall_min + overall_max) / 2
    half_range = (overall_max - overall_min) / 2 * 1.1  # 10% padding

    limit = (centre - half_range, centre + half_range)

    for ax, (i, j), title in zip(axes, projections, titles):
        ax.set_title(title)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel(['X', 'Y', 'Z'][i], labelpad=1)
        ax.set_ylabel(['X', 'Y', 'Z'][j], labelpad=-15)

        ax.set_xlim(limit)
        ax.set_ylim(limit)

        # --- PLATFORM ---
        corners = platform_box_coords()
        faces = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
                 (2, 3, 7, 6), (1, 2, 6, 5), (3, 0, 4, 7)]
        for face in faces:
            coords = corners[np.array(face)][:, [i, j]]
            coords = np.vstack([coords, coords[0]])
            ax.plot(coords[:, 0], coords[:, 1], 'k-', linewidth=1)

        # --- ARMS ---
        for arm in all_arms:
            start = arm["attachment_point"]
            end = arm["end_point"]
            ax.plot([start[i], end[i]], [start[j], end[j]],
                    color=arm["color"], label=arm["label"])

        # --- COG ---
        ax.plot(0, 0, 'kx', markersize=8, label='CoG')
        ax.text(0, 0, '  CoG', va='bottom', ha='left', fontsize=10, color='black')

        # --- Legend ---
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



# === DRAW 3D PLOT ===
def plot_drone_3D(arms):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("3D View of Drone Cross Section")

    all_arms = []
    for arm in arms:
        all_arms.extend(generate_attachments(arm))

    # --- PLATFORM ---
    corners = platform_box_coords()
    faces = [
        [0,1,2,3], [4,5,6,7],  # bottom, top
        [0,1,5,4], [2,3,7,6],  # front, back
        [1,2,6,5], [3,0,4,7]   # right, left
    ]
    face_polys = [corners[face] for face in faces]
    ax.add_collection3d(Poly3DCollection(face_polys, facecolors='gray', linewidths=1, edgecolors='k', alpha=0.4))

    for arm in all_arms:
        start = arm["attachment_point"]
        end = arm["end_point"]
        ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
                color=arm["color"], label=arm["label"])

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