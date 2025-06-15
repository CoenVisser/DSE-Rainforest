import numpy as np

from optimizers import Arms, Spines, Diameter
from linear import get_bumper_properties, get_foot_properties



m = 1.7 # Mass of bark [kg]
d_prop = 0.1778
delta_guard = 0.01  # Guard thickness [m]

material_properties = {
    "E_hook": 70e9,                      # Elastic modulus [Pa]
    "v_hook": 0.29,                      # Poisson's ratio [-]
    "sigma_yield_hook": 290e6,          # Yield strength [Pa]
    "density_hook": 7000,               # Density [kg/m^3]
    "m_hook": 0.00012644910430698917,    # Mass of the hook [kg]

    "E_spine": 40e9,                    # Elastic modulus [Pa]
    "density_spine": 1400,              # Density [kg/m^3]
    "sigma_yield_spine": 280e6,         # Yield strength [Pa]

    "E_bumper": 40e9,                   # Elastic modulus [Pa]
    "density_bumper": 1400,             # Density [kg/m^3]
    "sigma_yield_bumper": 280e6,        # Yield strength [Pa]

    "E_tree": 0.6e9,                    # Elastic modulus [Pa]
    "v_tree": 0.4,                      # Poisson's ratio [-]
    "sigma_yield_tree": 150e6,           # Yield strength [Pa]

    "E_arm": 40e9,
    "density_arm": 1400,
    "sigma_yield_arm": 280e6,

    "E_foot": 40e9,                    # Elastic modulus [Pa]
    "density_foot": 1400,              # Density [kg/m^3]
    "sigma_yield_foot": 280e6,         # Yield strength [Pa]
}

geometrical_properties = {
    "l_hook": 0.023,                    # Length of the hooks [m]
    "d_hook": 1e-3,                     # Diameter of the hooks [m]
    "n_hook": 20,                       # Number of hooks [-]
    "R_tip": 20e-6,                     # Radius of curvature of the spine [m]

    "l_spine": 0.3,                     # Length of the spine to origin [m]
    "d_spine": 0.005,                   # Diameter of the spine [m]
    "alpha_spine": 20,                 # Angle with respect to the symmetry plane [deg]
    "beta_spine": 10,                  # Angle with respect to the horizontal plane [deg]
    "n_spine": 2,                      # Number of spines [-]
    "spacing_spine": 0.15,            # Spacing between spines [m]

    "l_bumper": 0.3,                   # Length of the bumper to origin [m]
    "d_bumper": 0.005,                 # Diameter of the bumper [m]
    "alpha_bumper": 0,               # Angle with respect to the symmetry plane [deg]
    "beta_bumper": 10,                # Angle with respect to the horizontal plane [deg]
    "n_bumper": 1,                    # Number of bumpers [-]

    "l_foot": 0.15,                   
    "d_foot": 0.005,
    "alpha_foot": 0,
    "beta_foot": 10,
    "n_foot": 2,
    "spacing_foot": 0.1,
    
    "l_arm": 0.5,
    "d_arm_outer": 0.02,
    "d_arm_inner": 0.015,
    "alpha_arm": 0.1,
    "beta_arm": 0.1,

    "c_prop_h": 0.33,                 # Horizontal clearance of the propeller [-]
    "c_prop_v": 0.33,                 # Vertical clearance of the propeller [-]
    "d_prop": d_prop+delta_guard,                 # Diameter of the propeller [m]
    "h_prop": 0.01,
    "n_prop": 4,                      # Number of propellers [-]
    "P_max": 1301,
    "rpm_max": 34173,
    "min_thickness": 0.0025,
    "min_clearance": 0.0875,
    "thrust_loss": 0.03,

    "l_cg": 0.03,                     # Distance from surface to centre of mass [m]
    "l_platform": 0.1,      #Length of main body
    "w_platform": 0.062,    #Width of main body
    "h_platform": 0.059,     #Height of main body

    "alpha": 15,                      # Adhesion load angle [deg]

    "c_prop_v_outer": 0.10,
    "c_prop_h_outer": 0.10,
    "n_load": 2,
    "sf_forces_spumper": 1.25,
    "sf_forces_arm": 2,
}

bark_properties = {
    "m": m,                     # Mass of bark [kg]
    "W": m * 9.81,          # Weight of bark [N]
    "thrust_to_weight_ratio": 2,  # Thrust to weight ratio [-]
}

x0_arms = ["l_arm", "d_arm_outer", "d_arm_inner",
           "alpha_arm", "beta_arm", "n_prop"]

bounds_arms = [(0.01, 0.4),
               (0.005, 0.1),
               (0.005, 0.1),
               (0, 90),
               (0, 45),
               (4, 8)
]

x0_spines = ["l_spine", "alpha_spine", "beta_spine", "n_hook"] 

bounds_spines = [(0.01, 0.5),  # l_spine
                 (5, 30),     # alpha_spine
                 (10, 45),     # beta_spine
                 (1, 64),     # n_hook
]

x0_diameter = ["d_spine", "d_bumper", "d_foot"]

bounds_diameter = [(0.005, 0.1),  # d_spine
                   (0.005, 0.1),   # d_bumper
                   (0.005, 0.1)    # d_foot

]

print("Starting optimization...")

arms_optimizer = Arms(
    material_properties=material_properties,
    geometrical_properties=geometrical_properties,
    bark_properties=bark_properties,
    x0_arms=x0_arms,
    bounds_arms=bounds_arms
)

arms_optimizer.optimise()

geometrical_properties = arms_optimizer.geo

x0_bumper = ["l_bumper", "alpha_bumper", "beta_bumper"]
x0_foot = ["l_foot", "alpha_foot", "beta_foot"]

geometrical_properties['l_bumper'], geometrical_properties['beta_bumper'], geometrical_properties['x_bumper'], geometrical_properties['y_bumper'], geometrical_properties['z_bumper'] = get_bumper_properties(**geometrical_properties)
geometrical_properties['alpha_foot'], geometrical_properties['beta_foot'], geometrical_properties['x_foot'], geometrical_properties['y_foot'], geometrical_properties['z_foot'] = get_foot_properties(**geometrical_properties)

geometry_optimizer = Spines(
    material_properties=material_properties,
    geometrical_properties=geometrical_properties,
    bark_properties=bark_properties,
    x0_geometry=x0_spines,
    bounds_geometry=bounds_spines
)

geometry_optimizer.optimise()

geometrical_properties = geometry_optimizer.geo

diameter_optimizer = Diameter(
    material_properties=material_properties,
    geometrical_properties=geometrical_properties,
    bark_properties=bark_properties,
    x0_diameter=x0_diameter,
    bounds_diameter=bounds_diameter
)

diameter_optimizer.optimise()

geometrical_properties = diameter_optimizer.geo

print("Optimised Geometrical Properties:")

for key in x0_arms + x0_spines + x0_diameter:
    print(f"{key}: {geometrical_properties[key]}")

print("Arms Properties:")
print(geometrical_properties["l_arm"])
print(geometrical_properties["alpha_arm"])
print(geometrical_properties["beta_arm"])
print("------------------------------")
print("Spines Properties:")
print(geometrical_properties["l_spine"])
print(geometrical_properties["alpha_spine"])
print(geometrical_properties["beta_spine"])
print("------------------------------")
print("Bumper Properties:")
print(geometrical_properties["l_bumper"])
print(geometrical_properties["alpha_bumper"])
print(geometrical_properties["beta_bumper"])
print("------------------------------")
print("Foot Properties:")
print(geometrical_properties["l_foot"])
print(geometrical_properties["alpha_foot"])
print(geometrical_properties["beta_foot"])
print("------------------------------")
print("Hook Properties:")
print(geometrical_properties["n_hook"])



# bumper_properties = {
#     "side": "positive",
#     "length": geometrical_properties["l_bumper"],
#     "diameter": geometrical_properties["d_bumper"],
#     "alpha": geometrical_properties["alpha_bumper"],
#     "beta": geometrical_properties["beta_bumper"],
#     "number": geometrical_properties["n_bumper"],
#     "spacing": geometrical_properties["spacing_bumper"]
# }

# spine_properties = {
#     "side": "positive",
#     "length": geometrical_properties["l_spine"],
#     "diameter": geometrical_properties["d_spine"],
#     "alpha": geometrical_properties["alpha_spine"],
#     "beta": geometrical_properties["beta_spine"],
#     "number": geometrical_properties["n_spine"],
#     "spacing": geometrical_properties["spacing_spine"]
# }

# arm_properties = {
#     "side": "negative",
#     "length": geometrical_properties["l_arm"],
#     "outer_diameter": geometrical_properties["d_arm_outer"],
#     "inner_diameter": geometrical_properties["d_arm_inner"],
#     "alpha": geometrical_properties["alpha_arm"],
#     "beta": geometrical_properties["beta_arm"],
#     "number": geometrical_properties["n_prop"]
# }

# bumper_endpoint = end_point(geometrical_properties["l_cg"])