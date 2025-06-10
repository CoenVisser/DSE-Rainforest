#Propeller Sizing
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

#=======================================================
#Propeller Arm Properties
#=======================================================

# Propeller Arm Properties
E_arm = 40*10**9
density_arm = 1400
sigma_yield_arm = 280*10**6

# Propeller Arm Geometry
l_arm = 0.5
d_arm_outer = 0.02
d_arm_inner = 0.015
alpha_arm = 0.1
beta_arm = 0.1

# Maybe use a square geometry for manufacturability

#Propeller Arm Constants
c_prop_h = 0.33 #0.1
c_prop_v = 0.33 #0.1
d_prop = 0.15
beta_prop = 0
n_prop = 4

#========================================================
# Sizing Parameters
#========================================================
min_thickness = 0.0025
min_clearance = 0.0875
min_prop_clearance = 0.10 #%
n_buckling = 0.25
sf_forces = 2
sf_design = 1.25

thrust_loss = 0.03 #Amount of thrust that may be lost due to arm deflection

#Use Platform Parameters for prop clearance
#w_box
#l_box

#=======================================================
# Bark Properties
#=======================================================

m = 2.7
W = m*9.81*sf_design
thrust_to_weight_ratio = 2

#=======================================================
#Functions
#=======================================================
#The weight of the propulsion system at each arm is neglected
def get_max_shear(W, thrust_to_weight_ratio, n_prop):
    F_shear = W*thrust_to_weight_ratio/n_prop
    return sf_forces*F_shear

def get_max_normal_force(W, thrust_to_weight_ratio, n_prop):
    F_normal = W*(thrust_to_weight_ratio - 1)/n_prop
    return sf_forces*F_normal

def get_max_bending_moment(F_shear, l_arm):
    M_bending = F_shear * l_arm
    return M_bending

def get_torsion_moment(F_shear, l_arm, alpha_arm):
    T_torsion = F_shear * l_arm * np.sin(np.deg2rad(alpha_arm))
    return T_torsion

def get_max_shear_stress_torsion(T_torsion, d_arm_outer, d_arm_inner):
    J_cross_section = np.pi/32 * (d_arm_outer**4 - d_arm_inner**4)
    tau_torsion = T_torsion*(d_arm_outer/2)/ J_cross_section
    return tau_torsion

def get_max_shear_stress_bending(F_shear, d_arm_outer, d_arm_inner):
    t = d_arm_outer - d_arm_inner
    I = np.pi * (d_arm_outer**4 - d_arm_inner**4) / 64
    Q = 2/3 * (d_arm_outer**3 - d_arm_inner**3)
    tau_bending = F_shear * Q/ (I *t)
    return tau_bending

def get_max_normal_stress(F_normal, d_arm_outer, d_arm_inner):
    A = np.pi * (d_arm_outer**2 - d_arm_inner**2) / 4
    sigma_normal = F_normal / A
    return sigma_normal

def get_max_bending_stress(M_bending, d_arm_outer, d_arm_inner):
    I = np.pi * (d_arm_outer**4 - d_arm_inner**4) / 64
    r_max = d_arm_outer /2
    sigma_bending = M_bending * r_max / I
    return sigma_bending

def get_mass_arm(d_arm_outer, d_arm_inner, l_arm, density_arm):
    A = np.pi * (d_arm_outer**2 - d_arm_inner**2) / 4
    V = A * l_arm
    mass_arm = V * density_arm
    return mass_arm

#Include what if one arm fails
#Look into horizontal force of arm

#=======================================================
# Optimization
#=======================================================

x0 = [l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop]

bounds = [(0.01, 0.4), (0.005, 0.1), (0.005, 0.1), (0, 90), (0, 90), (4, 8)]

#========================================================
#Objective function and constraints
#========================================================
args_obj =  (density_arm)
args1 =     (W, thrust_to_weight_ratio)
args2 =     (E_arm, n_buckling)
args3 =     (W, thrust_to_weight_ratio)
args4 =     (min_clearance)
args5 =     (min_thickness)
args6 =     (min_prop_clearance)
args7 =     (min_prop_clearance)
args8 =     (E_arm, thrust_loss, W, thrust_to_weight_ratio)
args10 =    (W, thrust_to_weight_ratio)


def objective(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    density_arm = args_obj
    mass = n_prop*get_mass_arm(d_arm_outer, d_arm_inner, l_arm, density_arm)
    return mass

#Can not fail in maximum normal stress
def constraint1(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    W, thrust_to_weight_ratio = args
    F_normal = get_max_normal_force(W, thrust_to_weight_ratio, n_prop)
    sigma_normal = get_max_normal_stress(F_normal, d_arm_outer, d_arm_inner)
    M_bending = get_max_bending_moment(F_normal, l_arm)
    sigma_bending = get_max_bending_stress(M_bending, d_arm_outer, d_arm_inner)
    sigma_max = sigma_normal + sigma_bending
    return sigma_yield_arm - sigma_max
    
#Can not buckle
def constraint2(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    E_arm, n_buckling = args
    I = np.pi * (d_arm_outer**4 - d_arm_inner**4) / 64
    F_allowable = n_buckling*np.pi**2 * E_arm * I/ l_arm**2
    F_normal = get_max_normal_force(W, thrust_to_weight_ratio, n_prop)
    return F_allowable - F_normal

#Can not fail in maximum shear stress
def constraint3(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    W, thrust_to_weight_ratio = args
    F_shear = get_max_shear(W, thrust_to_weight_ratio, n_prop)
    tau_bending = get_max_shear_stress_bending(F_shear, d_arm_outer, d_arm_inner)
    T_torsion = get_torsion_moment(F_shear, l_arm, alpha_arm)
    tau_torsion = get_max_shear_stress_torsion(T_torsion, d_arm_outer, d_arm_inner)
    tau_max = tau_bending + tau_torsion
    return sigma_yield_arm - tau_max


#Beta should not hinder airflow
def constraint4(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    min_clearance = args
    z_arm = l_arm * np.sin(np.deg2rad(beta_arm))
    return z_arm - min_clearance

#The thickness should be sufficient
def constraint5(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    min_thickness = args
    t = d_arm_outer - d_arm_inner
    return t - min_thickness

#Propellers should not touch in y-direction
def constraint6(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    min_prop_clearance = args
    prop_radius = d_prop / 2
    min_distance = prop_radius*(1 + min_prop_clearance) #Already includes half for one side
    y_arm = l_arm * np.sin(np.deg2rad(alpha_arm))
    return y_arm - min_distance

#Propeller should not touch in x-direction
def constraint7(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    min_prop_clearance = args
    prop_radius = d_prop / 2
    min_distance = prop_radius*(1 + min_prop_clearance) #Already includes half for one side
    x_arm = l_arm * np.cos(np.deg2rad(alpha_arm))
    return x_arm - min_distance

#Deflection
def constraint8(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    E_arm, thrust_loss, W, thrust_to_weight_ratio = args
    max_angle_deflection = np.arctan([np.sqrt(1-(1-thrust_loss)**2)/(1-thrust_loss)])
    print(max_angle_deflection)
    F_shear = get_max_shear(W, thrust_to_weight_ratio, n_prop)
    I = np.pi * (d_arm_outer**4 - d_arm_inner**4) / 64
    theta = F_shear*l_arm**2 /(2*E_arm*I)
    return max_angle_deflection - theta  # Ensure deflection is within limits

#Principal stress
def constraint10(x, args):
    l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop = x
    W, thrust_to_weight_ratio = args
    F_normal = get_max_normal_force(W, thrust_to_weight_ratio, n_prop)
    sigma_normal = get_max_normal_stress(F_normal, d_arm_outer, d_arm_inner)
    M_bending = get_max_bending_moment(F_normal, l_arm)
    sigma_bending = get_max_bending_stress(M_bending, d_arm_outer, d_arm_inner)
    sigma_max = sigma_normal + sigma_bending
    F_shear = get_max_shear(W, thrust_to_weight_ratio, n_prop)
    tau_bending = get_max_shear_stress_bending(F_shear, d_arm_outer, d_arm_inner)
    T_torsion = get_torsion_moment(F_shear, l_arm, alpha_arm)
    tau_torsion = get_max_shear_stress_torsion(T_torsion, d_arm_outer, d_arm_inner)
    tau_max = tau_bending + tau_torsion
    # Calculate Von Mises stress
    sigma_principal = sigma_max/2 + np.sqrt((sigma_max/2)**2 + tau_max**2)
    return sigma_yield_arm - sigma_principal

constraints = [
    {'type': 'ineq', 'fun': lambda x: constraint1(x, args1)},
    {'type': 'ineq', 'fun': lambda x: constraint2(x, args2)},
    {'type': 'ineq', 'fun': lambda x: constraint3(x, args1)},
    {'type': 'ineq', 'fun': lambda x: constraint4(x, args4)},
    {'type': 'ineq', 'fun': lambda x: constraint5(x, args5)},
    {'type': 'ineq', 'fun': lambda x: constraint6(x, args6)},
    {'type': 'ineq', 'fun': lambda x: constraint7(x, args6)},
    {'type': 'ineq', 'fun': lambda x: constraint8(x, args8)},
    {'type': 'ineq', 'fun': lambda x: constraint10(x, args1)}
    ]

def optimize_propeller(x0, args, bounds, constraints):
    result = minimize(lambda x: objective(x, args), x0, bounds=bounds, constraints=constraints, method='SLSQP', options={'disp': True})
    return result

if __name__ =="__main__":
    result = optimize_propeller(x0, args_obj, bounds, constraints)
    if result.success:
        print("Optimization successful!")
        print("Optimal parameters:", result.x)
        print("Minimum mass:", result.fun)
    else:
        print("Optimization failed:", result.message)

#Look into Von Mises stress
