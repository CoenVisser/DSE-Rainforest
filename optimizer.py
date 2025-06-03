#Hooking Mechanism Force Plot
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

#======================================================================
# Plot parameters
#======================================================================

num = 1000                   #Plotting variable

#======================================================================
# Hook Properties
#======================================================================

# Hook Geometry
l_hook = 0.023               # Length of the hooks [m]
d_hook = 0.254* 10**(-3)              # Diameter of the hooks [m]
n_hook = 30                # Number of hooks [-]
R_tip = 20* 10**(-6)        # Radius of curvature of the spine [m]

# Hook Material
E_m = 200* 10**9                # Elastic modulus [Pa]
v_m = 0.29                      # Poisson's ratio [-]
sigma_yield = 290* 10**6        # Yield strength [Pa]

#=======================================================================
# Spine Properties
#=======================================================================

# Spine Geometry
l_spine = 0.7                   # Length of the spine to cg [m]
d_spine = 0.01                  # Diameter of the spine [m]
alpha_spine = 20                # Angle of the spine with respect to the symmetry plane [degrees]
beta_spine = 10                 # Angle of the spine with respect to the horizontal plane [degrees]
n_spine = 2                     # Number of spines [-]

# Spine Material
...

#=======================================================================
# Bumper Properties
#=======================================================================

# Bumper Geometry
l_bumper = 0.1                  # Length of the bumper to cg [m]
d_bumper = 0.01                 # Diameter of the bumper [m]
alpha_bumper = 20               # Angle of the bumper with respect to the symmetry plane [degrees]
beta_bumper = 10                # Angle of the bumper with respect to the horizontal plane [degrees]
n_bumper = 2                    # Number of bumpers [-]

# Bumper Material
...

#=======================================================================
#Propeller Properties
#=======================================================================

c_prop = 0.50                   #Clearance of the propeller [-]
d_prop = 0.25                   #Diameter of the propeller [m]
l_prop = np.sqrt(2*(d_prop*(1+c_prop)))                       #Length of the propeller arm for double symmetric quadcopter[m]
alpha_prop =   np.arctan(d_prop*(1+c_prop), d_prop*(1+c_prop))                 #Angle of the propeller with the symmetry plane [degrees]
beta_prop =   0                  #Angle of the propeller with the horizontal plane [degrees]
n_prop = 4                       #Number of propellers [-]

#=======================================================================
# Centre of Mass Properties
#=======================================================================

l_cg = 0.1                      # Distance from the surface to the center of mass [m]

#========================================================================
# Loading Properties
#========================================================================

alpha = 15                      # Adhesion load angle [degrees]

#=======================================================================
# Tree Properties
#=======================================================================

mu_asp = 0.20                   # Coefficient of friction [-]
Ex_asp = 9.8 *10**9             # Young's modulus [Pa]
v_asp = 0.4                     # Poisson's ratio [-]

#========================================================================
# Bark Properties
#========================================================================

m = 2.7                         # Mass of the bark [kg]
W = m*9.81                      # Weight of the bark [N]

x0 = [l_spine, alpha_spine, l_bumper, alpha_bumper]
args_obj = (beta_spine, beta_bumper)
args_cst1 = (c_prop, d_prop)

def Fs(W, n_hook):
    return W / n_hook

def Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook):
    Fn = - W*(l_cg + l_spine*np.cos(np.deg2rad(alpha_spine))*np.sin(np.deg2rad(beta_spine)))/(l_spine*np.cos(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)) + l_bumper*np.cos(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)))/n_hook
    return Fn

def objective(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper = x
    beta_spine, beta_bumper = args
    height = l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)) + l_bumper*np.cos(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper))
    width = np.maximum(l_spine*np.sin(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)), l_bumper*np.sin(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper)))
    return height * width

def constraint1(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper = x
    c_prop, d_prop = args
    height = l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)) + l_bumper*np.cos(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper))
    min_height = (2+c_prop)*d_prop
    return min_height - height

def constraint2(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper = x
    c_prop, d_prop = args
    width = np.maximum(l_spine*np.sin(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)), l_bumper*np.sin(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper)))
    min_width = (2+c_prop)*d_prop
    return min_width - width

