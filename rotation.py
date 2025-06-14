import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R, Slerp
import trimesh
from shapely.geometry import Polygon

#Does not take into account width --> Slightly alters
#==================================
# Inputs
#==================================

#Hooks
x_hooks = 0.02 #Length of the hook array
alpha_hooks = 0 # From optimizer
alpha_hooks_post = 0
beta_hooks = 0
beta_hooks_post = 0
H = [0,0,0] #From optimizer
C_H = np.array([0.052, 0.003, 0.031]) #Can split into C_H +-
l_hooks_post = 1.0

#Bumper
B2 = [0,0,0] #From optimizer
alpha_bumper = 0 #From optimizer
alpha_bumper_post = 0
beta_bumper = 0 #From optimizer
beta_bumper_post = 0
r = 0 #From Christa
C_B2 = np.array([-0.052, 0.003, 3.1]) #Can split into C_B2 +-
l_b2_post = 1

#Interface
theta = 30 #Rotation of array with respect to beta_0


#Propeller
d_prop = 0.17

#Plotting
zoom = 1
steps = 100
num_circle_points = 20


#=========================================================================
H_incl_hooks = H + np.array([0.5*x_hooks*np.cos(theta)*np.cos(alpha_hooks),
                             0.5*x_hooks*np.cos(theta)*np.sin(alpha_hooks),
                             0.5*x_hooks*np.sin(theta)])

B2_incl_bumper = B2 + np.array([r*np.cos(beta_bumper+theta)*np.cos(alpha_bumper),
                                r*np.cos(beta_bumper+theta)*np.sin(alpha_bumper),
                                r*np.sin(beta_bumper+theta)])

def rotation_simulation(attached_shape, origin, length_post, beta_post, alpha_post, theta, length_attachment, width_attachment):
    # === USER SETTINGS ===
    #attached_shape = 'cylinder'  # Options: 'square' or 'cylinder'

    # === SIMULATION PARAMETERS ===
    #origin = C_H #Connection point
    start_euler = [0, beta_post, alpha_post] #There are two versions pre and post
    end_euler = [0, beta_post+theta*np.cos(alpha_post), alpha_post+np.sin(alpha_post)]

    # === ROTATION SETUP ===
    key_times = [0, 1]
    key_rots = R.from_euler('ZYX', [start_euler, end_euler], degrees=True)
    slerp = Slerp(key_times, key_rots)
    times = np.linspace(0, 1, steps)
    rotations = slerp(times)

    # === ATTACHED SHAPE GEOMETRY ===

    # Square plate (in YZ plane)
    square_local = np.array([
        [0, -length_attachment / 2, -width_attachment / 2],
        [0,  length_attachment / 2, -width_attachment / 2],
        [0,  length_attachment / 2,  width_attachment / 2],
        [0, -length_attachment / 2,  width_attachment / 2]
    ])

    theta = np.linspace(0, 2 * np.pi, num_circle_points)

    # Circle in x-y plane at z = 0 and z = L (aligned along z)
    cyl_radius = 0.5*width_attachment
    cyl_length = length_attachment
    circle1 = np.stack([cyl_radius * np.cos(theta),
                        cyl_radius * np.sin(theta),
                        np.zeros_like(theta)], axis=1)
    circle2 = circle1.copy()
    circle2[:, 2] += cyl_length

    cylinder_faces = []
    for i in range(num_circle_points - 1):
        cylinder_faces.append([circle1[i], circle1[i+1], circle2[i+1], circle2[i]])
    cylinder_faces.append([circle1[-1], circle1[0], circle2[0], circle2[-1]])
    cylinder_faces.append(circle1[::-1])  # cap 1
    cylinder_faces.append(circle2)        # cap 2

    # === PLOTTING ===
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.ion()

    for R_i in rotations:
        ax.cla()
        ax.set_xlim(-zoom, zoom)
        ax.set_ylim(-zoom, zoom)
        ax.set_zlim(-zoom, zoom)
        ax.set_box_aspect([1, 1, 1])
        ax.set_title(f"Rotating Line with Attached {attached_shape.capitalize()}")

        # --- LINE ---
        line_vector = R_i.apply([length_post*np.cos(np.deg2rad(alpha_post))*np.cos(np.deg2rad(beta_post)),
                                length_post*np.sin(np.deg2rad(alpha_post))*np.cos(np.deg2rad(beta_post)),
                                length_post*np.sin(np.deg2rad(beta_post))])
        end_point = origin + line_vector
        ax.plot(
            [origin[0], end_point[0]],
            [origin[1], end_point[1]],
            [origin[2], end_point[2]],
            'r-', linewidth=3
        )

        if attached_shape == 'square':
            square_rotated = R_i.apply(square_local)
            square_translated = square_rotated + end_point
            square = Poly3DCollection([square_translated], alpha=0.6, color='cyan', edgecolor='k')
            ax.add_collection3d(square)

        elif attached_shape == 'cylinder':
            # Apply rotation and translate to tip of line
            faces_rotated = [R_i.apply(face) + end_point for face in cylinder_faces]
            cyl = Poly3DCollection(faces_rotated, alpha=0.7, facecolor='skyblue', edgecolor='k')
            ax.add_collection3d(cyl)

        else:
            raise ValueError("Invalid shape type. Use 'square' or 'cylinder'.")

        plt.draw()
        plt.pause(0.05)

    plt.ioff()
    plt.show()