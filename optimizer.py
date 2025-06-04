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
E_hook = 200* 10**9                # Elastic modulus [Pa]
v_hook = 0.29                      # Poisson's ratio [-]
sigma_yield_hook = 290* 10**6        # Yield strength [Pa]

#=======================================================================
# Spine Properties
#=======================================================================

# Spine Geometry
l_spine = 0.7                   # Length of the spine to cg [m]
d_spine = 0.01                  # Diameter of the spine [m]
alpha_spine = 20                # Angle of the spine with respect to the symmetry plane [degrees]
beta_spine = 0                 # Angle of the spine with respect to the horizontal plane [degrees]
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
beta_bumper = 0                # Angle of the bumper with respect to the horizontal plane [degrees]
n_bumper = 2                    # Number of bumpers [-]

# Bumper Material
...

#=======================================================================
#Propeller Properties
#=======================================================================

c_prop_h = 0.50                     # Horizontal clearance of the propeller [-]
c_prop_v = 0.50                     # Vertical clearance of the propeller [-]
d_prop = 0.25                       # Diameter of the propeller [m]
alpha_prop = np.arctan2(d_prop*0.5*(1+c_prop_h), d_prop*0.5*(1+c_prop_v))   # Angle of the propeller with the symmetry plane [degrees]
beta_prop =   0                                                             # Angle of the propeller with the horizontal plane [degrees]
n_prop = 4                          # Number of propellers [-]

#=======================================================================
# Centre of Mass Properties
#=======================================================================

l_cg = 0.05                      # Distance from the surface to the center of mass [m]

#========================================================================
# Loading Properties
#========================================================================

alpha = 15                      # Adhesion load angle [degrees]

#=======================================================================
# Tree Properties
#=======================================================================

mu_tree = 0.2                   # Coefficient of friction [-]
E_tree = 9.8 *10**9             # Young's modulus [Pa]
v_tree = 0.4                     # Poisson's ratio [-]
sigma_yield_tree = 250*10**6

#========================================================================
# Bark Properties
#========================================================================

m = 2.7                         # Mass of the bark [kg]
W = m*9.81                      # Weight of the bark [N]

x0 = [l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook]
bounds = [(0.01, 1.0), (0, 90), (0.01, 1.0), (0, 90), (1, 10000000000)]  # Bounds for l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook
args_obj = (beta_spine, beta_bumper)

def get_Fs(W, n_hook):
    return W / n_hook

def get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook):
    Fn = - W*(l_cg + l_spine*np.cos(np.deg2rad(alpha_spine))*np.sin(np.deg2rad(beta_spine)))/(l_spine*np.cos(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)) + l_bumper*np.cos(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)))/n_hook
    return Fn

def get_Fmax(v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree):
    E_tot = 1/((1-v_hook**2)/E_hook + (1-v_tree**2)/E_tree)            #Not sure about the 1/ part

    F_max = (np.pi*sigma_yield_tree/(1-2*v_tree))**3 * 9*R_tip**2/(2*E_tot**2)
    # todo: asperity failing
    return F_max

def get_Smax(F_tot, l, d):
    return 32*F_tot*l*d/(np.pi*d**4)

def objective(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook = x
    beta_spine, beta_bumper = args
    height = l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)) + l_bumper*np.cos(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper))
    width = max(l_spine*np.sin(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)), l_bumper*np.sin(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper)))
    return height * width

args1 = (c_prop_v, d_prop, beta_spine, beta_bumper)

def constraint1a(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook = x
    c_prop_v, d_prop, beta_spine, beta_bumper = args
    height = l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine))
    min_height = (1+c_prop_v/2)*d_prop
    return height - min_height

def constraint1b(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook = x
    c_prop_v, d_prop, beta_spine, beta_bumper = args
    height = l_bumper*np.cos(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper))
    min_height = (1+c_prop_v/2)*d_prop
    return height - min_height

args2 = (c_prop_h, d_prop, beta_spine, beta_bumper)

def constraint2(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook = x
    c_prop_h, d_prop, beta_spine, beta_bumper = args
    width = np.maximum(l_spine*np.sin(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)), l_bumper*np.sin(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper)))
    min_width = (2+c_prop_h)*d_prop
    return width - min_width

args3 = (W, l_cg, beta_spine, beta_bumper, v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree)

def constraint3(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook = x
    W, l_cg, beta_spine, beta_bumper, v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree = args
    Fs = 2*get_Fs(W, n_hook)
    Fn = 2*get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook)
    F_tot = np.sqrt(Fs**2 + Fn**2)
    F_max = get_Fmax(v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree)
    return F_max - F_tot

args4 = (W, l_cg, beta_spine, beta_bumper, sigma_yield_hook, l_hook, d_hook)

def constraint4(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook = x
    W, l_cg, beta_spine, beta_bumper, sigma_yield_hook, l_hook, d_hook = args
    Fs = get_Fs(W, n_hook)
    Fn = get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook)
    F_tot = np.sqrt(Fs**2 + Fn**2)
    Smax = get_Smax(F_tot, l_hook, d_hook)
    return sigma_yield_hook - Smax

args5 = (W, l_cg, beta_spine, beta_bumper, alpha)

def constraint5(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook = x
    W, l_cg, beta_spine, beta_bumper, alpha = args
    Fs = get_Fs(W, n_hook)
    Fn = get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook)
    return np.deg2rad(alpha) + Fn/Fs

constraints = [
    {'type': 'ineq', 'fun': lambda x: constraint1a(x, args1)},
    {'type': 'ineq', 'fun': lambda x: constraint1b(x, args1)},
    {'type': 'ineq', 'fun': lambda x: constraint2(x, args2)},
    {'type': 'ineq', 'fun': lambda x: constraint3(x, args3)},
    # {'type': 'ineq', 'fun': lambda x: constraint4(x, args4)},
    {'type': 'ineq', 'fun': lambda x: constraint5(x, args5)}
]

def optimize_hook(x0, args, bounds, constraints):
    result = minimize(lambda x: objective(x, args), x0, bounds=bounds, constraints=constraints, method='SLSQP', options={'disp': True})
    return result

if __name__ == "__main__":
    result = optimize_hook(x0, args_obj, bounds, constraints)
    print(result.message)
    print("Optimized parameters:", result.x)

    print(get_Fs(W, result.x[4]))
    print(get_Fn(l_cg, result.x[0], result.x[2], result.x[1], beta_spine, result.x[3], beta_bumper, W, result.x[4]))