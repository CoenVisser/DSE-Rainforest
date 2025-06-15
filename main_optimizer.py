import numpy as np

from optimizers import Arms, Spines, Diameter
from linear import get_bumper_properties, get_foot_properties
from arm_coordinates import end_point, length_and_angle
from droneplot import plot_drone_2D, plot_drone_3D



m = 1.44162 # Mass of bark [kg]
d_prop = 0.1778
delta_guard = 0.01  # Guard thickness [m]
l_platform = 0.15  # Length of main body [m]
w_platform = 0.062  # Width of main body [m]
h_platform = 0.059  # Height of main body [m]

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
    "spacing_spine": 0.1,            # Spacing between spines [m]

    "l_bumper": 0.3,                   # Length of the bumper to origin [m]
    "d_bumper": 0.005,                 # Diameter of the bumper [m]
    "alpha_bumper": 0,               # Angle with respect to the symmetry plane [deg]
    "beta_bumper": 10,                # Angle with respect to the horizontal plane [deg]
    "n_bumper": 1,                    # Number of bumpers [-]

    "l_foot": 0.21,                   
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
    "l_platform": l_platform,      #Length of main body
    "w_platform": w_platform,    #Width of main body
    "h_platform": h_platform,     #Height of main body
    "attachment_foot": np.array([0.052, 0.03, 0.5*h_platform]),
    "attachment_bumper": np.array([0.47*l_platform, 0, -0.5*h_platform]),
    "attachment_spines": np.array( [0.052, 0.03, 0.5*h_platform]),
    "attachment_arms": np.array([0.47*l_platform, 0.03, -0.5*h_platform]),

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
                   (0.01, 0.1),   # d_bumper
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

geometrical_properties["x_arms"], geometrical_properties["y_arms"], geometrical_properties["z_arms"] = end_point(
    geometrical_properties["l_cg"],
    geometrical_properties["h_platform"],
    "negative",
    geometrical_properties["l_arm"],
    geometrical_properties["alpha_arm"],
    geometrical_properties["beta_arm"]
    )

geometrical_properties["x_spines"], geometrical_properties["y_spines"], geometrical_properties["z_spines"] = end_point(
    geometrical_properties["l_cg"],
    geometrical_properties["h_platform"],
    "positive",
    geometrical_properties["l_spine"],
    geometrical_properties["alpha_spine"],
    geometrical_properties["beta_spine"]
)

geometrical_properties["end_point_spine"] = np.array([geometrical_properties["x_spines"],
                                                        geometrical_properties["y_spines"],
                                                        geometrical_properties["z_spines"]])

geometrical_properties["end_point_arms"] = np.array([geometrical_properties["x_arms"],
                                                        geometrical_properties["y_arms"],
                                                        geometrical_properties["z_arms"]])

geometrical_properties["end_point_foot"] = np.array([geometrical_properties["x_foot"],
                                                        geometrical_properties["y_foot"],
                                                        geometrical_properties["z_foot"]])

geometrical_properties["end_point_bumper"] = np.array([geometrical_properties["x_bumper"],
                                                        geometrical_properties["y_bumper"],
                                                        geometrical_properties["z_bumper"]])

print("End Points [x, y, z]:")
print(f"Spine: {geometrical_properties['end_point_spine']}")
print(f"Foot: {geometrical_properties['end_point_foot']}")
print(f"Arms: {geometrical_properties['end_point_arms']}")
print(f"Bumper: {geometrical_properties['end_point_bumper']}")

l_spine_true, alpha_spine_true, beta_spine_true = length_and_angle(
    geometrical_properties["end_point_spine"],
    geometrical_properties["attachment_spines"]
)

l_arms_true, alpha_arms_true, beta_arms_true = length_and_angle(
    geometrical_properties["end_point_arms"],
    geometrical_properties["attachment_arms"]
)

l_bumper_true, alpha_bumper_true, beta_bumper_true = length_and_angle(
    geometrical_properties["end_point_bumper"],
    geometrical_properties["attachment_bumper"]
)

l_foot_true, alpha_foot_true, beta_foot_true = length_and_angle(
    geometrical_properties["end_point_foot"],
    geometrical_properties["attachment_foot"]
)

mass = geometrical_properties["n_spine"]*l_spine_true*geometrical_properties["d_spine"]**2*np.pi/4*material_properties["density_spine"] + \
         geometrical_properties["n_bumper"]*l_bumper_true*geometrical_properties["d_bumper"]**2*np.pi/4*material_properties["density_bumper"] + \
            geometrical_properties["n_foot"]*l_foot_true*geometrical_properties["d_foot"]**2*np.pi/4*material_properties["density_foot"] + \
            geometrical_properties["n_prop"]*l_arms_true*(geometrical_properties["d_arm_outer"]-geometrical_properties["d_arm_inner"])**2*np.pi/4*material_properties["density_arm"] + \
            geometrical_properties["n_hook"]*material_properties["m_hook"]*4 + 0.003*(2*l_platform*w_platform + 2*l_platform*h_platform + 2*w_platform*h_platform) * material_properties["density_arm"]

print(f"Total mass: {mass:.4f} kg")

points = [
    {
        "length": l_arms_true,
        "alpha": alpha_arms_true,
        "beta": beta_arms_true,
        "color": "blue",
        "label": "Propeller"
    },
    {
        "length": l_bumper_true,
        "alpha": alpha_bumper_true,
        "beta": beta_bumper_true,
        "color": "orange",
        "label": "Bumper"
    },
    {
        "length": l_foot_true,
        "alpha": alpha_foot_true,
        "beta": beta_foot_true,
        "color": "green",
        "label": "Foot"
    },
    {
        "length": l_spine_true,
        "alpha": alpha_spine_true,
        "beta": beta_spine_true,
        "color": "red",
        "label": "Spine"
    }
]

print("Length and Angles:")
for point in points:
    print(f"{point['label']}: Length = {point['length']:.4f} m, Alpha = {point['alpha']:.2f} deg, Beta = {point['beta']:.2f} deg")