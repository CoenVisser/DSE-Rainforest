from Optimizers import Arms, Geometry, Diameter


m = 1.7 # Mass of bark [kg]

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
    "density_bumper": 1500,             # Density [kg/m^3]
    "sigma_yield_bumper": 280e6,        # Yield strength [Pa]

    "E_tree": 0.6e9,                    # Elastic modulus [Pa]
    "v_tree": 0.4,                      # Poisson's ratio [-]
    "sigma_yield_tree": 150e6,           # Yield strength [Pa]

    "E_arm": 40e9,
    "density_arm": 1400,
    "sigma_yield_arm": 280e6,
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
    "alpha_bumper": 20,               # Angle with respect to the symmetry plane [deg]
    "beta_bumper": 10,                # Angle with respect to the horizontal plane [deg]
    "n_bumper": 2,                    # Number of bumpers [-]
    "spacing_bumper": 0.1,           # Spacing between bumpers [m]

    "l_brake": 0.1,
    "d_brake": 0.005,
    "alpha_brake": 0,
    "beta_brake": -10.3063,

    "l_arm": 0.5,
    "d_arm_outer": 0.02,
    "d_arm_inner": 0.015,
    "alpha_arm": 0.1,
    "beta_arm": 0.1,

    "c_prop_h": 0.33,                 # Horizontal clearance of the propeller [-]
    "c_prop_v": 0.33,                 # Vertical clearance of the propeller [-]
    "d_prop": 0.1778,                 # Diameter of the propeller [m]
    "n_prop": 4,                      # Number of propellers [-]
    "P_max": 1301,
    "rpm_max": 34173,
    "min_thickness": 0.0025,
    "min_clearance": 0.0875,
    "thrust_loss": 0.03,

    "l_cg": 0.05,                     # Distance from surface to centre of mass [m]

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
    "l_platform": 0.1,      #Length of main body
    "w_platform": 0.062,    #Width of main body
    "h_platform": 0.059     #Height of main body
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

x0_geometry = ["l_spine", "alpha_spine", "beta_spine",
               "l_bumper", "alpha_bumper", "beta_bumper",
               "n_hook", "l_cg"] 

bounds_geometry = [(0.01, 0.5),  # l_spine
                   (5, 30),     # alpha_spine
                   (10, 45),     # beta_spine
                   (0.01, 0.5),  # l_bumper
                   (5, 30),     # alpha_bumper
                   (10, 45),     # beta_bumper
                   (1, 64),     # n_hook
                   (0.03, 0.1)  # l_cg
]

x0_diameter = ["d_spine", "d_bumper"]

bounds_diameter = [(0.005, 0.1),  # d_spine
                   (0.005, 0.1)   # d_bumper
]

arms_optimizer = Arms(
    material_properties=material_properties,
    geometrical_properties=geometrical_properties,
    bark_properties=bark_properties,
    x0_arms=x0_arms,
    bounds_arms=bounds_arms
)

arms_optimizer.optimise()

geometrical_properties = arms_optimizer.geo

geometry_optimizer = Geometry(
    material_properties=material_properties,
    geometrical_properties=geometrical_properties,
    bark_properties=bark_properties,
    x0_geometry=x0_geometry,
    bounds_geometry=bounds_geometry
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

for key in x0_arms + x0_geometry + x0_diameter:
    print(f"{key}: {geometrical_properties[key]}")