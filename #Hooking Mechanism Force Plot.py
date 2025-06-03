#Hooking Mechanism Force Plot
import numpy as np
import matplotlib.pyplot as plt

#======================================================================
# Plot parameters
#======================================================================

num = 1000                   #Plotting variable

#======================================================================
# Hook Properties
#======================================================================

# Hook Geometry
l_hook = 0.023                          # Length of the hooks [m]
d_hook = 0.254* 10**(-3)                # Diameter of the hooks [m]
n_hook = 30                             # Number of hooks [-]
R_tip = 20* 10**(-6)                    # Radius of curvature of the spine [m]

# Hook Material - Stainless Steel
E_hook = 200* 10**9                        # Elastic modulus [Pa]
v_hook = 0.29                              # Poisson's ratio [-]
sigma_yield_hook = 290* 10**6                # Yield strength [Pa]
rho_hook = 7790                                 # Density [kg/m^3]

#=======================================================================
# Spine Properties
#=======================================================================

# Spine Geometry
l_spine = 0.7                           # Length of the spine to cg [m]
d_spine = 0.01                          # Diameter of the spine [m]
alpha_spine = 20                        # Angle of the spine with respect to the symmetry plane [degrees]
beta_spine = 10                         # Angle of the spine with respect to the horizontal plane [degrees]
n_spine = 2                             # Number of spines [-]

# Spine Material - Aluminium
E_spine = 25* 10**9          # Elastic modulus [Pa]
sigma_yield_spine = 270* 10**6        # Yield strength [Pa]
rho_spine = 2700                      # Density [kg/m^3]


#=======================================================================
# Bumper Properties
#=======================================================================

# Bumper Geometry
l_bumper = 0.1                  # Length of the bumper to cg [m]
d_bumper = 0.01                 # Diameter of the bumper [m]
alpha_bumper = 20               # Angle of the bumper with respect to the symmetry plane [degrees]
beta_bumper = 10                # Angle of the bumper with respect to the horizontal plane [degrees]
n_bumper = 2                    # Number of bumpers [-]

# Bumper Material - EDPM Rubber
# Include bounciness for impact absorption
E_bumper = 6 * 10**6                # Elastic modulus [Pa]
rho_bumper = 1430                   # Density [kg/m^3]

#=======================================================================
# Centre of Mass Properties
#=======================================================================

l_cg = 0.1                      # Distance from the surface to the center of mass [m]

#=======================================================================
# Propeller Properties
#=======================================================================
clong_prop = 0.50                  #Cloning of the propeller [-]
clat_prop = 0.50                   #Clearance of the propeller [%]
d_prop = 0.25                   #Diameter of the propeller [m]
l_prop = np.sqrt((d_prop*(1+clong_prop))**2+(d_prop*(1+clat_prop))**2)                                         #Length of the propeller arm for double symmetric quadcopter[m]
alpha_prop =   np.arctan(d_prop/2*(1+clat_prop), d_prop/2*(1+clong_prop))                  #Angle of the propeller with the symmetry plane [degrees]
beta_prop =   0                  #Angle of the propeller with the horizontal plane [degrees]
n_prop = 4                       #Number of propellers [-]

#========================================================================
# Loading Properties
#========================================================================

alpha = 15                      # Adhesion load angle [degrees]
dynamic_load_factor = 10          # Dynamic load factor [-]

#=======================================================================
# Tree Properties
#=======================================================================

mu_tree = 0.20                   # Coefficient of friction [-]
E_tree = 9.8 *10**9             # Young's modulus [Pa]
v_tree = 0.4                     # Poisson's ratio [-]

#========================================================================
# Bark Properties
#========================================================================

m = 2.7                         # Mass of the bark [kg]
W = m*9.81                      # Weight of the bark [N]

#=======================================================================
# Force Calculations
#=======================================================================

Fs = m*9.81/n_hook              # Tangential force per hook [N]
Fnh = - m*9.81*(l_cg + l_spine*np.cos(np.deg2rad(alpha_spine))*np.sin(np.deg2rad(beta_spine)))/(l_spine*np.cos(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)) + l_bumper*np.cos(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)))/n_hook
                                # Normal force per hook [N]

F_tot = np.sqrt(Fs**2 + Fnh**2) # Total force per hook [N]
gamma = np.arctan2(Fs, Fnh)     # Angle of the force vector with respect to the horizontal [radians]

sigma_max = 32*F_tot*l_hook*d_hook/(np.pi*d_hook**4)        # Maximum induced stress [Pa]

E_tot = 1/((1-v_m**2)/E_m + (1-v_asp**2)/Ex_asp)            #Not sure about the 1/ part

F_tree = (np.pi*sigma_max/(1-2*v_asp))**3 * 9*R_tip**2/(2*E_tot**2)
F_mat = np.pi/4 * d_hook**2 * sigma_yield
F_max = min(F_tree, F_mat)                                  #Minimum sizing force

F_actual = F_tot*dynamic_load_factor

#=======================================================================
#Impact Calculations
#=======================================================================

lift_to_weight_ratio_impact = 1.5
delta_t_impact = 0.4
moment_of_intertia = m*(l_bumper*np.cos(np.deg2rad(beta_bumper))*np.sin(np.deg2rad(alpha_bumper)))**2

acceleration_impact = 0.5*(0.5*np.pi)/(delta_t_impact**2)
omega_impact = (0.5*np.pi)/delta_t_impact
v_impact = omega_impact * (l_bumper*np.cos(np.deg2rad(beta_bumper))*np.sin(np.deg2rad(alpha_bumper)))

F_impact = m*v_impact/delta_t_impact
sigma_impact = F_impact/(l_bumper*d_bumper)


#=======================================================================
#Plotting
#=======================================================================

fig, ax = plt.subplots(subplot_kw={'projection':'polar'})

theta = np.linspace(-np.deg2rad(alpha), np.arctan(mu_asp)+0.5*np.pi, num)
r = [F_max]*num
r_actual = [F_actual]*num
ax.fill_between(theta, r, color='green', alpha=0.5)
ax.plot(theta, r_actual, color='purple', alpha=0.5)

theta_upper = [np.arctan(mu_asp)+0.5*np.pi] * num
r_upper = np.linspace(0, F_max, num)
ax.plot(theta_upper, r_upper, color='red')

theta_lower = [-np.deg2rad(alpha)] * num
r_lower = np.linspace(0, F_max, num)
ax.plot(theta_lower, r_lower, color='blue')

ax.grid(True)
ax.set_title('Force Plot for Hooking Mechanism')
plt.show()