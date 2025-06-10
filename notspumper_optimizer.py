#Hooking Mechanism Force Plot
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

#======================================================================
# Hook Properties
#======================================================================

# Hook Material 
E_hook = 70* 10**9                      # Elastic modulus [Pa]
v_hook = 0.29                           # Poisson's ratio [-]
sigma_yield_hook = 290* 10**6           # Yield strength [Pa]
density_hook = 7000                     # Density [kg/m^3]

# Hook Geometry
l_hook = 0.023                          # Length of the hooks [m]
d_hook = 1* 10**(-3)                    # Diameter of the hooks [m]
n_hook = 20                             # Number of hooks [-]
R_tip = 20* 10**(-6)                    # Radius of curvature of the spine [m]

m_hook = density_hook*np.pi*(d_hook/2)**2*l_hook  # Mass of the hook [kg]

#=======================================================================
# Spine Properties
#=======================================================================

# Spine Material - Flax Fibers
E_spine = 70*10**9                      # Elastic modulus [Pa]
density_spine = 1400
sigma_yield_spine = 280*10**6           # Yield strength [Pa]

# Spine Geometry
l_spine = 0.3                           # Length of the spine to cg [m]
d_spine = 0.005                          # Diameter of the spine [m]
alpha_spine = 20                        # Angle of the spine with respect to the symmetry plane [degrees]
beta_spine = 10                         # Angle of the spine with respect to the horizontal plane [degrees]
n_spine = 2                             # Number of spines [-]
spacing_spine = 0.15                    # Spacing between 2 spines [m]

#=======================================================================
# Bumper Properties
#=======================================================================

# Bumper Material - Flax Fibers
E_bumper = 70*10**9                      # Elastic modulus [Pa]
density_bumper = 1500                    # Density [kg/m^3] 
sigma_yield_bumper = 280*10**6           # Yield strength [Pa]

# Bumper Geometry
l_bumper = 0.3                          # Length of the bumper to cg [m]
d_bumper = 0.005                         # Diameter of the bumper [m]
alpha_bumper = 20                       # Angle of the bumper with respect to the symmetry plane [degrees]
beta_bumper = 10                        # Angle of the bumper with respect to the horizontal plane [degrees]
n_bumper = 2                            # Number of bumpers [-]
spacing_bumper = 0.1                    # Spacing between 2 bumpers [m]

#=======================================================================
# Propeller Properties
#=======================================================================

# Propeller Geometry
c_prop_h = 0.33                         # Horizontal clearance of the propeller [-]
c_prop_v = 0.33                         # Vertical clearance of the propeller [-]
d_prop = 0.15                           # Diameter of the propeller [m]
beta_prop =   0                         # Angle of the propeller with the horizontal plane [degrees]
n_prop = 4                              # Number of propellers [-]

#=======================================================================
# Centre of Mass Properties
#=======================================================================

l_cg = 0.05                             # Distance from the surface to the center of mass [m]

#========================================================================
# Loading Properties
#========================================================================

alpha = 15                              # Adhesion load angle [degrees]

#=======================================================================
# Tree Properties
#=======================================================================

# Tree Material - Bark
mu_tree = 0.2                           # Coefficient of friction [-]
E_tree = 0.6 *10**9                     # Young's modulus [Pa]
v_tree = 0.4                            # Poisson's ratio [-]
sigma_yield_tree = 150*10**6            # Yield strength [Pa]

#========================================================================
# Bark Properties
#========================================================================

m = 2.7                                 # Mass of the bark [kg]
W = m*9.81                              # Weight of the bark [N]

#========================================================================
# Relevant Margins
#========================================================================
spacing_beta_spine = 10
spacing_beta_bumper = 10
c_prop_v_outer = 0.10
c_prop_h_outer = 0.10
n_load = 2
sf_forces = 1.25
sf_design = 1.2


#========================================================================
# Functions
#========================================================================

def get_Fs(W, n_hook):
    return W / n_hook * sf_design

def get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook):
    Fn = - W*(l_cg + l_spine*np.cos(np.deg2rad(alpha_spine))*np.sin(np.deg2rad(beta_spine)))/(l_spine*np.cos(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)) + l_bumper*np.cos(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)))/n_hook
    return Fn * sf_forces

def get_Fmax(v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree):
    E_tot = 1/((1-v_hook**2)/E_hook + (1-v_tree**2)/E_tree)
    F_max = (np.pi*sigma_yield_tree/(1-2*v_tree))**3 * 9*R_tip**2/(2*E_tot**2)
    return F_max #Dont add safety factor, it is already in Ftot

def get_Smax(F_tot, l, d):
    return 32*F_tot*l*d/(np.pi*d**4) * sf_forces

def get_area(l_spine, alpha_spine, l_bumper, alpha_bumper, beta_spine, beta_bumper):
    height = l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)) + l_bumper*np.cos(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper))
    width = max(l_spine*np.sin(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)), l_bumper*np.sin(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper)))
    return height * width

def get_mass(l_spine, d_spine, density_spine, l_bumper, d_bumper, density_bumper, n_hook, m_hook, n_spine, n_bumper):
    mass_spine = n_spine*l_spine*(np.pi*(d_spine/2)**2)*density_spine
    mass_bumper = n_bumper*l_bumper*(np.pi*(d_bumper/2)**2)*density_bumper
    mass_hook = 4*n_hook*m_hook
    return mass_spine+mass_bumper+mass_hook, mass_spine, mass_bumper, mass_hook

#========================================================================
# Optimization Setup
#========================================================================

# Initial parameters - parameters to be optimized
x0 = [l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper]

# Bounds for the parameters
bounds = [(0.01, 0.5), (5, 30), (0.01, 0.5), (5, 30), (1, 65), (0.03, 0.10), (0, 45), (0, 45)]

# Initial mass and area calculations
initial_mass = get_mass(l_spine, d_spine, density_spine, l_bumper, d_bumper, density_bumper, n_hook, m_hook, n_spine, n_bumper)[0]
initial_area = get_area(l_spine, alpha_spine, l_bumper, alpha_bumper, beta_spine, beta_bumper)

print(f"Initial mass: {initial_mass:.4f} kg")
print(f"Initial area: {initial_area:.4f} m^2")

# Arguments for the objective function and constraints
args_obj = (m_hook, density_spine, density_bumper, n_spine, n_bumper, d_spine, d_bumper, initial_mass, initial_area)
args1 = (c_prop_v, c_prop_v_outer, d_prop)
args2 = (c_prop_v,c_prop_v_outer, d_prop)
args3 = (W, v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree)
args4 = (W, sigma_yield_hook, l_hook, d_hook)
args5 = (W, alpha)
args6 = ()
args7 = (beta_prop, spacing_beta_spine)
args8 = (beta_prop, spacing_beta_bumper)
args9 = (spacing_spine)
args10 = (spacing_bumper)

#========================================================================
# Objective function and constraints
#========================================================================

# Objective function to minimize: weighted sum of area and mass
def objective(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    m_hook, density_spine, density_bumper, n_spine, n_bumper, d_spine, d_bumper, initial_mass, initial_area = args
    area = get_area(l_spine, alpha_spine, l_bumper, alpha_bumper, beta_spine, beta_bumper)
    mass = get_mass(l_spine, d_spine, density_spine, l_bumper, d_bumper, density_bumper, n_hook, m_hook, n_spine, n_bumper)[0]
    w_area = 0.5
    w_mass = 1 - w_area
    return w_area * area/initial_area + w_mass * mass/initial_mass

# Height of the spine must be greater than the properller
def constraint1(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    c_prop_v, c_prop_v_outer, d_prop = args
    height = l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine))
    min_height = (1+c_prop_v/2+c_prop_v_outer/2)*d_prop
    return height - min_height

# Height of the bumper must be greater than the properller
def constraint2(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    c_prop_v, c_prop_v_outer, d_prop = args
    height = l_bumper*np.cos(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper))
    min_height = (1+c_prop_v/2+c_prop_v_outer/2)*d_prop
    return height - min_height

# Force must be less than the maximum force that the tree can withstand
def constraint3(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    W, v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree = args
    Fs = get_Fs(W, n_hook)
    Fn = get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook)
    F_tot = np.sqrt(Fs**2 + Fn**2)
    F_max = get_Fmax(v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree)
    return F_max - F_tot

# Stress in the hook must be less than the yield strength of the hook material
def constraint4(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    W, sigma_yield_hook, l_hook, d_hook = args
    Fs = get_Fs(W, n_hook)
    Fn = get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook)
    F_tot = np.sqrt(Fs**2 + Fn**2)
    Smax = get_Smax(F_tot, l_hook, d_hook)
    return sigma_yield_hook - Smax

# Contact angle of the hooks must be greater than the adhesion angle
def constraint5(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    W, alpha = args
    Fs = get_Fs(W, n_hook)
    Fn = get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook)
    return np.deg2rad(alpha) + Fn/(Fs + 1e-8)

# Bumper and spine must be aligned in the horizontal plane
def constraint6(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    return l_spine*np.sin(np.radians(beta_spine)) - l_bumper*np.sin(np.radians(beta_bumper))

# Beta_spine must be at least 10 degrees more than beta_propeller
def constraint7(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    beta_prop, spacing_beta_spine = args
    return beta_spine - spacing_beta_spine - beta_prop

# Beta_bumper must be at least 10 degrees more than beta_propeller
def constraint8(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    beta_prop, spacing_beta_bumper = args
    return beta_bumper - spacing_beta_bumper - beta_prop

# The spacing between the spines must be spacing_spine
def constraint9(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    spacing_spine = args
    spacing = 2 * l_spine * np.sin(np.radians(alpha_spine)) * np.cos(np.radians(beta_spine))
    return spacing - spacing_spine

# The spacing between the bumpers must be spacing_bumper
def constraint10(x, args):
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper = x
    spacing_bumper = args
    spacing = 2 * l_bumper * np.sin(np.radians(alpha_bumper)) * np.cos(np.radians(beta_bumper))
    return spacing - spacing_bumper

constraints = [
    {'type': 'ineq', 'fun': lambda x: constraint1(x, args1)},
    {'type': 'ineq', 'fun': lambda x: constraint2(x, args2)},
    {'type': 'ineq', 'fun': lambda x: constraint3(x, args3)},
    {'type': 'ineq', 'fun': lambda x: constraint4(x, args4)},
    {'type': 'ineq', 'fun': lambda x: constraint5(x, args5)},
    {'type': 'eq', 'fun': lambda x: constraint6(x, args6)},
    {'type': 'ineq', 'fun': lambda x: constraint7(x, args7)},
    {'type': 'ineq', 'fun': lambda x: constraint8(x, args8)},
    {'type': 'eq', 'fun': lambda x: constraint9(x, args9)},
    {'type': 'eq', 'fun': lambda x: constraint10(x, args10)}
]

def optimize_hook(x0, args, bounds, constraints):
    result = minimize(lambda x: objective(x, args), x0, bounds=bounds, constraints=constraints, method='SLSQP', options={'disp': True})
    return result

if __name__ == "__main__":
    result = optimize_hook(x0, args_obj, bounds, constraints)
    print(result.message)
    print("Optimized parameters:")
    print(f"l_spine: {result.x[0]:.4f} m")
    print(f"alpha_spine: {result.x[1]:.4f} degrees")
    print(f"l_bumper: {result.x[2]:.4f} m")
    print(f"alpha_bumper: {result.x[3]:.4f} degrees")
    print(f"n_hook: {result.x[4]:.4f} hooks")
    print(f"l_cg: {result.x[5]:.4f} m")
    print(f"beta_spine: {result.x[6]:.4f} degrees")
    print(f"beta_bumper: {result.x[7]:.4f} degrees")

    # print("Calculated Forces:")
    # print(f"Fs: {get_Fs(W, result.x[4]):.4f} N")
    # print(f"Fn: {get_Fn(result.x[5], result.x[0], result.x[2], result.x[1], result.x[6], result.x[3], result.x[7], W, result.x[4]):.4f} N")
    # print(f"F_max: {get_Fmax(v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree):.4f} N")

    print(f"Area: {get_area(result.x[0], result.x[1], result.x[2], result.x[3], result.x[6], result.x[7]):.4f} m^2")
    print(f"Mass: {get_mass(result.x[0], d_spine, density_spine, result.x[2], d_bumper, density_bumper, result.x[4], m_hook, n_spine, n_bumper)[0]:.4f} kg")
    print(f"Mass of spine: {get_mass(result.x[0], d_spine, density_spine, result.x[2], d_bumper, density_bumper, result.x[4], m_hook, n_spine, n_bumper)[1]:.4f} kg")
    print(f"Mass of bumper: {get_mass(result.x[0], d_spine, density_spine, result.x[2], d_bumper, density_bumper, result.x[4], m_hook, n_spine, n_bumper)[2]:.4f} kg")
    print(f"Mass of hooks: {get_mass(result.x[0], d_spine, density_spine, result.x[2], d_bumper, density_bumper, result.x[4], m_hook, n_spine, n_bumper)[3]:.4f} kg")