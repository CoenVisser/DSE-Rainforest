#Hooking Mechanism Force Plot
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

#======================================================================
#USE THE INITIAL PROPERTIES OBTAINED FROM OPTIMIZER.PY
#======================================================================
#PUT IN THE MARGINS FROM ELIA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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
n_hook = 32.6179                             # Number of hooks [-]
R_tip = 20* 10**(-6)                    # Radius of curvature of the spine [m]

m_hook = density_hook*np.pi*(d_hook/2)**2*l_hook  # Mass of the hook [kg]

#=======================================================================
# Spine Properties
#=======================================================================

# Spine Material - Flax Fibers
E_spine = 20*10**9                      # Elastic modulus [Pa]
density_spine = 1500
sigma_yield_spine = 280*10**6           # Yield strength [Pa]

# Spine Geometry
l_spine = 0.2004                           # Length of the spine to cg [m]
d_spine = 0.05                          # Diameter of the spine [m]
alpha_spine = 22.3638                        # Angle of the spine with respect to the symmetry plane [degrees]
beta_spine = 10.3818                         # Angle of the spine with respect to the horizontal plane [degrees]
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
l_bumper = 0.1924                          # Length of the bumper to cg [m]
d_bumper = 0.05                         # Diameter of the bumper [m]
alpha_bumper = 15.3415                       # Angle of the bumper with respect to the symmetry plane [degrees]
beta_bumper = 10.8026                        # Angle of the bumper with respect to the horizontal plane [degrees]
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

l_cg = 0.03                             # Distance from the surface to the center of mass [m]

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
sf_forces = 2
sf_design = 1.5

#========================================================================
# Functions
#========================================================================

def get_Fs(W, n_hook):
    return W / n_hook

def get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook):
    Fn = - W*(l_cg + l_spine*np.cos(np.deg2rad(alpha_spine))*np.sin(np.deg2rad(beta_spine)))/(l_spine*np.cos(np.deg2rad(alpha_spine))*np.cos(np.deg2rad(beta_spine)) + l_bumper*np.cos(np.deg2rad(alpha_bumper))*np.cos(np.deg2rad(beta_bumper)))/n_hook
    return Fn

def get_Fmax(v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree):
    E_tot = 1/((1-v_hook**2)/E_hook + (1-v_tree**2)/E_tree)
    F_max = (np.pi*sigma_yield_tree/(1-2*v_tree))**3 * 9*R_tip**2/(2*E_tot**2)
    return F_max

def get_Smax(F_tot, l, d):
    return 32*F_tot*l*d/(np.pi*d**4)

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
x0 = [d_spine, d_bumper]

# Bounds for the parameters
bounds = [(0.005, 0.1), (0.005, 0.1)]

# Initial mass and area calculations
initial_mass = get_mass(l_spine, d_spine, density_spine, l_bumper, d_bumper, density_bumper, n_hook, m_hook, n_spine, n_bumper)[0]
initial_area = get_area(l_spine, alpha_spine, l_bumper, alpha_bumper, beta_spine, beta_bumper)

print(f"Initial mass: {initial_mass:.4f} kg")
print(f"Initial area: {initial_area:.4f} m^2")

# Arguments for the objective function and constraints
args_objb = (l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, m_hook, density_spine, density_bumper, n_spine, n_bumper, initial_mass, initial_area)
argsb1 = (l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, W, sigma_yield_spine, n_spine)
argsb2 = (l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, E_bumper, m)
argsb3 = (l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, m_hook, n_spine, E_spine)

#========================================================================
# Objective function and constraints
#========================================================================

# Objective function to minimize: weighted sum of area and mass
def objectiveb(x, args):
    d_spine, d_bumper = x
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, m_hook, density_spine, density_bumper, n_spine, n_bumper, initial_mass, initial_area = args
    area = get_area(l_spine, alpha_spine, l_bumper, alpha_bumper, beta_spine, beta_bumper)
    mass = get_mass(l_spine, d_spine, density_spine, l_bumper, d_bumper, density_bumper, n_hook, m_hook, n_spine, n_bumper)[0]
    w_area = 0.5
    w_mass = 1 - w_area
    return w_area * area/initial_area + w_mass * mass/initial_mass

# Stress in the spine must be less than the yield strength of the spine material
def constraintb1(x, args):
    d_spine, d_bumper = x
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, W, sigma_yield_spine, n_spine = args
    Fs = get_Fs(W, n_hook)
    Fn = get_Fn(l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook)
    F_tot = np.sqrt(Fs**2 + Fn**2)*n_spine
    S = F_tot/(np.pi * (d_spine/2)**2)
    return sigma_yield_spine - S

# The bumper must not buckle under a 1g load
def constraintb2(x, args):
    d_spine, d_bumper = x
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, E_bumper, m = args
    amoi_bumper = np.pi/4*(d_bumper/2)**4 
    Fbuck = 2.04*np.pi**2*E_bumper*amoi_bumper/(l_bumper**2)
    return Fbuck - m*9.81

# The spines must not deflect too much under the weight of the hooks
def constraintb3(x, args):
    d_spine, d_bumper = x
    l_spine, alpha_spine, l_bumper, alpha_bumper, n_hook, l_cg, beta_spine, beta_bumper, m_hook, n_spine, E_spine = args
    m_hooks = 4*n_hook*m_hook
    m_hooks_per_spine = m_hooks / n_spine
    amoi_spine = np.pi/4*(d_spine/2)**4
    deflection = m_hooks_per_spine * 9.81 * l_spine**3 / (3 * E_spine * amoi_spine)
    max_deflection = 0.1* l_spine  # Allowable deflection is 10% of the spine length
    return max_deflection - deflection

constraints = [
    {'type': 'ineq', 'fun': lambda x: constraintb1(x, argsb1)},
    {'type': 'ineq', 'fun': lambda x: constraintb2(x, argsb2)},
    {'type': 'ineq', 'fun': lambda x: constraintb3(x, argsb3)},
]

def optimize_hook(x0, args, bounds, constraints):
    result = minimize(lambda x: objectiveb(x, args), x0, bounds=bounds, constraints=constraints, method='SLSQP', options={'disp': True})
    return result

if __name__ == "__main__":
    result = optimize_hook(x0, args_objb, bounds, constraints)
    print(result.message)
    print("Optimized parameters:")
    print(f"d_spine: {result.x[0]:.4f} m")
    print(f"d_bumper: {result.x[1]:.4f} m")

    # print("Calculated Forces:")
    # print(f"Fs: {get_Fs(W, result.x[4]):.4f} N")
    # print(f"Fn: {get_Fn(result.x[5], result.x[0], result.x[2], result.x[1], result.x[6], result.x[3], result.x[7], W, result.x[4]):.4f} N")
    # print(f"F_max: {get_Fmax(v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree):.4f} N")

    print(f"Area: {get_area(l_spine, alpha_spine, l_bumper, alpha_bumper, beta_spine, beta_bumper):.4f} m^2")
    print(f"Mass: {get_mass(l_spine, result.x[0], density_spine, l_bumper, result.x[1], density_bumper, n_hook, m_hook, n_spine, n_bumper)[0]:.4f} kg")
    print(f"Mass of spine: {get_mass(l_spine, result.x[0], density_spine, l_bumper, result.x[1], density_bumper, n_hook, m_hook, n_spine, n_bumper)[1]:.4f} kg")
    print(f"Mass of bumper: {get_mass(l_spine, result.x[0], density_spine, l_bumper, result.x[1], density_bumper, n_hook, m_hook, n_spine, n_bumper)[2]:.4f} kg")
    print(f"Mass of hooks: {get_mass(l_spine, result.x[0], density_spine, l_bumper, result.x[1], density_bumper, n_hook, m_hook, n_spine, n_bumper)[3]:.4f} kg")