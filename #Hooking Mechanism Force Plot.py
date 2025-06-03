#Hooking Mechanism Force Plot
import numpy as np
import matplotlib.pyplot as plt

#======================================================================
# Plot parameters
#======================================================================

num = 100                   #Plotting variable

#======================================================================
# Hook Properties
#======================================================================

# Hook Geometry
l_hook = 0.02               # Length of the hooks [m]
d_hook = 0.001              # Diameter of the hooks [m]
n_hook = 120                # Number of hooks [-]
R_tip = 20* 10**(-6)        # Radius of curvature of the spine [m]

# Hook Material
E_m = 200* 10**9            # Elastic modulus [Pa]
v_m = 0.29                  # Poisson's ratio [-]
sigma_yield = 290* 10**6    # Yield strength [Pa]

#=======================================================================
# Spine Properties
#=======================================================================

# Spine Geometry
l_spine = 0.2               # Length of the spine to cg [m]
d_spine = 0.01              # Diameter of the spine [m]
alpha_spine = 20            # Angle of the spine with respect to the symmetry plane [degrees]
beta_spine = 10             # Angle of the spine with respect to the horizontal plane [degrees]
n_spine = 2                 # Number of spines [-]

# Spine Material
...

#=======================================================================
# Bumper Properties
#=======================================================================

# Bumper Geometry
l_bumper = 0.1              # Length of the bumper to cg [m]
d_bumper = 0.01             # Diameter of the bumper [m]
alpha_bumper = 20           # Angle of the bumper with respect to the symmetry plane [degrees]
beta_bumper = 10            # Angle of the bumper with respect to the horizontal plane [degrees]
n_bumper = 2                # Number of bumpers [-]

# Bumper Material
...

#=======================================================================
# Centre of Mass Properties
#=======================================================================

l_cg = 0.1                  # Distance from the surface to the center of mass [m]

#========================================================================
# Loading Properties
#========================================================================

alpha = 15                  # Adhesion load angle [degrees]

#=======================================================================
# Tree Properties
#=======================================================================

mu_asp = 0.20               # Coefficient of friction [-]
Ex_asp = 9.8 *10**9         # Young's modulus [Pa]
v_asp = 0.4                 # Poisson's ratio [-]

#========================================================================
# Bark Properties
#========================================================================

m = 2.7                     # Mass of the bark [kg]
W = m*9.81                  # Weight of the bark [N]

#=======================================================================
# Force Calculations
#=======================================================================

Fs = m*9.81/n_hook          # Tangential force per hook [N]
Fnb = m*9.81*(l_cg + l_hook*np.cos(np.deg2rad(alpha_spine))*np.sin(np.deg2rad(beta_spine)))/(l_spine*np.cos(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)) + l_bumper*np.cos(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)))/n_hook
Fnh = -Fnb

F_tot = np.sqrt(Fs**2 + Fnh**2)
gamma = np.arctan2(Fnb, Fnh)

sigma_max = 32*F_tot*l_hook*d_hook/(np.pi*d_hook**4)        #Maximum induced stress

E_tot = 1/((1-v_m**2)/E_m + (1-v_asp**2)/Ex_asp)            #Not sure about the 1/ part

f_tree = (np.pi*sigma_max/(1-2*v_asp))**3 * 9*R_tip**2/(2*E_tot**2)
f_mat = np.pi/4 * d_hook**2 * sigma_yield
f_max = min(f_tree, f_mat)                                  #Minimum sizing force

f_actual = F_tot/n_hook

#Plotting
fig, ax = plt.subplots(subplot_kw={'projection':'polar'})
theta = np.linspace(-np.deg2rad(alpha), np.arctan(mu_asp)+0.5*np.pi, num)
r = [f_max]*num
r_actual = [f_actual]*num
ax.plot(theta, r_actual, color='black', label='Sizing Force')
ax.fill_between(theta, r, color='green', alpha=0.5)
theta_upper = [np.arctan(mu_asp)+0.5*np.pi] * num
r_upper = np.linspace(0, f_max, num)
ax.plot(theta_upper, r_upper, color='red')

theta_lower = [-np.deg2rad(alpha)] * num
r_lower = np.linspace(0, f_max, num)
ax.fill_between(theta_lower, r_lower, color='blue')

ax.grid(True)
ax.set_title('Force Plot for Hooking Mechanism')
plt.show()




