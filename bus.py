import numpy as np
import matplotlib.pyplot as plt

# === PLATFORM DIMENSIONS ===
l_platform = 0.15
w_platform = 0.062
h_platform = 0.059
CoG_platform = [0, 0, 0]

# === LOWER EXTENSION BOX ===
h_lower_box = 0.01
CoG_lower_box = [0, 0, -0.5 * h_platform - 0.5 * h_lower_box]


# === BATTERY DIMENSIONS ===
l_battery = 0.1382
w_battery = 0.0562
h_battery = 0.0376
CoG_battery = [0, 0, 0.5 * h_platform - 0.5 * h_battery]

# === FLIGHT CONTROLLER DIMENSIONS ===
l_contr = 0.035
w_contr = 0.035
h_contr = 0.003
CoG_contr = [0.25 * l_platform, 0, -0.5 * h_platform + 0.5 * h_contr]

# === RASPBERRY PI DIMENSIONS ===
l_rasp = 0.065
w_rasp = 0.03
h_rasp = 0.005
CoG_rasp = [-0.25 * l_platform, 0, -0.5 * h_platform + 0.5 * h_rasp]

# === FUNCTION TO GET 8 CORNERS ===
def platform_coords(l, w, h, CoG):
    dx, dy, dz = l / 2, w / 2, h / 2
    cx, cy, cz = CoG
    corners = [
        [cx - dx, cy - dy, cz - dz], [cx + dx, cy - dy, cz - dz],
        [cx + dx, cy + dy, cz - dz], [cx - dx, cy + dy, cz - dz],
        [cx - dx, cy - dy, cz + dz], [cx + dx, cy - dy, cz + dz],
        [cx + dx, cy + dy, cz + dz], [cx - dx, cy + dy, cz + dz]
    ]
    return np.array(corners)

# === PLOT FUNCTION ===
def plot_platform():
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    titles = ['Top View (XY)', 'Side View (XZ)', 'Front View (YZ)']
    projections = [(0, 1), (0, 2), (1, 2)]

    # Define objects with colors
    objects = [
        (platform_coords(l_platform, w_platform, h_platform, CoG_platform), 'Platform', 'black'),
        (platform_coords(l_platform, w_platform, h_lower_box, CoG_lower_box), 'Attachment Area', 'purple'),
        (platform_coords(l_battery, w_battery, h_battery, CoG_battery), 'Battery', 'skyblue'),
        (platform_coords(l_contr, w_contr, h_contr, CoG_contr), 'Flight Controller', 'limegreen'),
        (platform_coords(l_rasp, w_rasp, h_rasp, CoG_rasp), 'Raspberry Pi', 'orange')
    ]

    # Box faces: list of 4-vertex faces
    faces = [(0, 1, 2, 3), (4, 5, 6, 7),
             (0, 1, 5, 4), (2, 3, 7, 6),
             (1, 2, 6, 5), (3, 0, 4, 7)]

    for ax, (i, j), title in zip(axes, projections, titles):
        plotted_segments = dict()

        for corners, label, color in objects:
            for face in faces:
                proj = corners[list(face)][:, [i, j]]
                edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
                for k1, k2 in edges:
                    p1 = tuple(np.round(proj[k1], 6))
                    p2 = tuple(np.round(proj[k2], 6))
                    key = tuple(sorted([p1, p2]))  # unordered edge

                    
                    # First time — solid line
                    ax.plot(*zip(p1, p2), linestyle='solid', color=color, linewidth=2, label=label)
                    plotted_segments[key] = True

        # CoG marker
        ax.plot(0, 0, 'kx', markersize=8, label='CoG')
        ax.text(0, 0, '  CoG', va='bottom', ha='left', fontsize=10, color='black')

        # Titles and labels
        ax.set_title(title)
        ax.set_xlabel(['X', 'X', 'Y'][axes.tolist().index(ax)])
        ax.set_ylabel(['Y', 'Z', 'Z'][axes.tolist().index(ax)])
        ax.axis('equal')
        ax.grid(True)

    # Clean up legend
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axes[0].legend(unique.values(), unique.keys(), loc='upper right')

    plt.tight_layout()
    plt.show()

# === RUN FUNCTION ===
plot_platform()
