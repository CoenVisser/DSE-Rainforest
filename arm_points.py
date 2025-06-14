#Create points in 3D space to get the angles and lengths for the arms
import numpy as np

#Obtain the properties from the other files
#Platform properties
l_platform = 0.15
w_platform = 0.062
h_platform = 0.059

#B1 Properties
l_initbumper = 0.1                  # Initial length of the bumper to CoG [m]
alpha_initbumper = 0               # Initial angle of the bumper with respect to the symmetry plane [degrees] - Zero
beta_initbumper = -10.3063                # Initial angle of the bumper with respect to the horizontal plane [degrees] - Opposite of beta_bumper

#B2 Properties
l_bumper = 0.2254                      # Length of the bumper to CoG [m]
alpha_bumper = 13.0318                   # Angle of the bumper with respect to the symmetry plane [degrees]
beta_bumper = 10.3063                    # Angle of the bumper with respect to the horizontal plane [degrees]

#H Properties
l_spine = 0.2322                               # Length of the spine to CoG [m]
alpha_spine = 19.1459                          # Angle of the spine with respect to the symmetry plane [degrees]
beta_spine = 10.0000                           # Angle of the spine with respect to the horizontal plane [degrees]

#Propeller Properties
l_prop = 1.63652041e-01                         # Length of the propeller arm for double symmetric quadcopter [m]
alpha_prop = 4.49999255e+01                     # Angle of the propeller with the symmetry plane [degrees]
beta_prop = 3.23215876e+01                      # Angle of the propeller with the horizontal plane [degrees]

d_prop = 0.1778
r_c_prop_guard = 0.01

#B1 calculation
l_initbumper = 1/np.cos(np.deg2rad(beta_initbumper)) * l_prop* np.cos(np.deg2rad(beta_prop)) * np.cos(np.deg2rad(alpha_prop)) + 0.5*d_prop + r_c_prop_guard
print(l_initbumper)

#The origin is at the middle of the box
CoG = np.array([0.0, 0.0, 0.0])  # Centre of Gravity in 3D space

B1 = CoG + np.array([l_initbumper*np.cos(np.deg2rad(alpha_initbumper))*np.cos(np.deg2rad(beta_initbumper)),
                     l_initbumper*np.sin(np.deg2rad(alpha_initbumper))*np.cos(np.deg2rad(beta_initbumper)),
                     l_initbumper*np.sin(np.deg2rad(beta_initbumper))])

B2 = CoG + np.array([l_bumper*np.cos(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)),
                     l_bumper*np.sin(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)),
                     l_bumper*np.sin(np.deg2rad(beta_bumper))])

H = CoG + np.array([l_spine*np.cos(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)),
                   l_spine*np.sin(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)),
                   l_spine*np.sin(np.deg2rad(beta_spine))])

Prop = CoG + np.array([l_prop*np.cos(np.deg2rad(alpha_prop))*np.cos(np.deg2rad(beta_prop)),
                       l_prop*np.sin(np.deg2rad(alpha_prop))*np.cos(np.deg2rad(beta_prop)),
                       l_prop*np.sin(np.deg2rad(beta_prop))])

B1_attachment = np.array([0.5*l_platform,0.5*w_platform,-0.5*h_platform])  # Attachment point for B1
B2_attachment = np.array([0.052, 0.03,0.5*h_platform])
H_attachment = np.array( [0.052,0.03,0.5*h_platform])
Prop_attachment = np.array([0,0,0])

def calculate_arm_lengths_and_angles():
    # Calculate lengths of the arms
    l_B1 = np.linalg.norm(B1 - B1_attachment)
    l_B2 = np.linalg.norm(B2 - B2_attachment)
    l_H = np.linalg.norm(H - H_attachment)
    l_Prop = np.linalg.norm(Prop - Prop_attachment)

    # Calculate angles of the arms with respect to the horizontal plane
    alpha_B1 = np.degrees(np.arctan2(B1[2] - B1_attachment[2], np.sqrt((B1[0] - B1_attachment[0])**2 + (B1[1] - B1_attachment[1])**2)))
    alpha_B2 = np.degrees(np.arctan2(B2[2] - B2_attachment[2], np.sqrt((B2[0] - B2_attachment[0])**2 + (B2[1] - B2_attachment[1])**2)))
    alpha_H = np.degrees(np.arctan2(H[2] - H_attachment[2], np.sqrt((H[0] - H_attachment[0])**2 + (H[1] - H_attachment[1])**2)))
    alpha_Prop = np.degrees(np.arctan2(Prop[2] - Prop_attachment[2], np.sqrt((Prop[0] - Prop_attachment[0])**2 + (Prop[1] - Prop_attachment[1])**2)))

    beta_B1 = np.degrees(np.arctan2(B1[1] - B1_attachment[1], B1[0] - B1_attachment[0]))
    beta_B2 = np.degrees(np.arctan2(B2[1] - B2_attachment[1], B2[0] - B2_attachment[0]))
    beta_H = np.degrees(np.arctan2(H[1] - H_attachment[1], H[0] - H_attachment[0]))
    beta_Prop = np.degrees(np.arctan2(Prop[1] - Prop_attachment[1], Prop[0] - Prop_attachment[0]))

    return (l_B1, l_B2, l_H, l_Prop, alpha_B1, alpha_B2, alpha_H, alpha_Prop, beta_B1, beta_B2, beta_H, beta_Prop)

print("Arm lengths and angles:")
lengths_and_angles = calculate_arm_lengths_and_angles()
print(f"Length B1: {lengths_and_angles[0]:.3f} m, Angle B1: {lengths_and_angles[4]:.2f} degrees, Beta B1: {lengths_and_angles[8]:.2f} degrees")
print(f"Length B2: {lengths_and_angles[1]:.3f} m, Angle B2: {lengths_and_angles[5]:.2f} degrees, Beta B2: {lengths_and_angles[9]:.2f} degrees")
print(f"Length H: {lengths_and_angles[2]:.3f} m, Angle H: {lengths_and_angles[6]:.2f} degrees, Beta H: {lengths_and_angles[10]:.2f} degrees")
print(f"Length Prop: {lengths_and_angles[3]:.3f} m, Angle Prop: {lengths_and_angles[7]:.2f} degrees, Beta Prop: {lengths_and_angles[11]:.2f} degrees")

print("Attachment points:")
print(f'CoG: {CoG}')
print(f'B1: {B1}')
print(f'B2: {B2}')
print(f'H: {H}')
print(f'Prop: {Prop}')

def end_point(CoG, length, alpha, beta):
    end_point = CoG + np.array([length*np.cos(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                                length*np.sin(np.deg2rad(alpha))*np.cos(np.deg2rad(beta)),
                                length*np.sin(np.deg2rad(beta))])
    return end_point

def length_and_angle(end_point, attachment_point):
    length = np.linalg.norm(end_point-attachment_point)
    alpha = np.degrees(np.arctan2(end_point[2]-attachment_point[2], np.sqrt((end_point[0]-attachment_point[0])**2+(end_point[1]-attachment_point[1])**2)))
    beta = np.degrees(np.arctan2(end_point[1]-attachment_point[1], end_point[0]-attachment_point[0]))
    return length, alpha, beta