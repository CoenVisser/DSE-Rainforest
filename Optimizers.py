import numpy as np
from scipy.optimize import minimize

class Functions:
    def get_Fs(self, W, n_hook, sf_forces_spumper, **kwargs):
        return W / n_hook  * sf_forces_spumper

    def get_Fn(self, l_cg, l_spine, l_bumper, alpha_spine, beta_spine, alpha_bumper, beta_bumper, W, n_hook, sf_forces_spumper, **kwargs):
        Fn = - W*(l_cg + l_spine*np.cos(np.radians(alpha_spine))*np.sin(np.radians(beta_spine))) / (
            l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)) +
            l_bumper*np.cos(np.radians(alpha_bumper))*np.cos(np.radians(beta_bumper))
        ) / n_hook
        return Fn * sf_forces_spumper

    def get_Fmax(self, v_hook, E_hook, sigma_yield_tree, R_tip, v_tree, E_tree, **kwargs):
        E_tot = 1 / ((1 - v_hook**2) / E_hook + (1 - v_tree**2) / E_tree)
        F_max = (np.pi * sigma_yield_tree / (1 - 2 * v_tree))**3 * 9 * R_tip**2 / (2 * E_tot**2)
        return F_max

    def get_Smax(self, F_tot, l, d, **kwargs):
        return 32 * F_tot * l * d / (np.pi * d**4)

    def get_area(self, l_spine, alpha_spine, l_bumper, alpha_bumper, beta_spine, beta_bumper, **kwargs):
        height = l_spine * np.cos(np.radians(alpha_spine)) * np.cos(np.radians(beta_spine)) + \
                 l_bumper * np.cos(np.radians(alpha_bumper)) * np.cos(np.radians(beta_bumper))
        width = max(
            l_spine * np.sin(np.radians(alpha_spine)) * np.cos(np.radians(beta_spine)),
            l_bumper * np.sin(np.radians(alpha_bumper)) * np.cos(np.radians(beta_bumper))
        )
        return height * width

    def get_mass(self, l_spine, d_spine, density_spine, l_bumper, d_bumper, density_bumper, n_hook, m_hook, n_spine, n_bumper, **kwargs):
        mass_spine = n_spine * l_spine * (np.pi * (d_spine / 2)**2) * density_spine
        mass_bumper = n_bumper * l_bumper * (np.pi * (d_bumper / 2)**2) * density_bumper
        mass_hook = 4 * n_hook * m_hook
        return mass_spine + mass_bumper + mass_hook, mass_spine, mass_bumper, mass_hook
    
    def get_max_shear(self, W, thrust_to_weight_ratio, n_prop, sf_forces_arm, **kwargs):
        F_shear = W*thrust_to_weight_ratio/n_prop
        return sf_forces_arm*F_shear

    def get_max_normal_force(self, W, thrust_to_weight_ratio, n_prop, sf_forces_arm, **kwargs):
        F_normal = W*(thrust_to_weight_ratio - 1)/n_prop
        return sf_forces_arm*F_normal

    def get_max_bending_moment(self, F_shear, l_arm, **kwargs):
        M_bending = F_shear * l_arm
        return M_bending

    def get_torsion_moment(self, F_shear, l_arm, alpha_arm, **kwargs):
        T_torsion = F_shear * l_arm * np.sin(np.radians(alpha_arm))
        return T_torsion

    def get_max_shear_stress_torsion(self, T_torsion, d_arm_outer, d_arm_inner, **kwargs):
        J_cross_section = np.pi/32 * (d_arm_outer**4 - d_arm_inner**4)
        tau_torsion = T_torsion*(d_arm_outer/2)/ J_cross_section
        return tau_torsion

    def get_max_shear_stress_bending(self, F_shear, d_arm_outer, d_arm_inner, **kwargs):
        t = d_arm_outer - d_arm_inner
        I = np.pi * (d_arm_outer**4 - d_arm_inner**4) / 64
        Q = 2/3 * (d_arm_outer**3 - d_arm_inner**3)
        tau_bending = F_shear * Q/ (I *t)
        return tau_bending

    def get_max_normal_stress(self, F_normal, d_arm_outer, d_arm_inner, **kwargs):
        A = np.pi * (d_arm_outer**2 - d_arm_inner**2) / 4
        sigma_normal = F_normal / A
        return sigma_normal

    def get_max_bending_stress(self, M_bending, d_arm_outer, d_arm_inner, **kwargs):
        I = np.pi * (d_arm_outer**4 - d_arm_inner**4) / 64
        r_max = d_arm_outer /2
        sigma_bending = M_bending * r_max / I
        return sigma_bending

    def get_propeller_torque(self, P_max, rpm_max, **kwargs):
        # Torque = Power / Angular Velocity
        omega_max = 2 * np.pi * rpm_max / 60  # Convert rpm to rad/s
        torque_max = P_max / omega_max
        return torque_max

    def get_propeller_torque_stress(self, torque_max, d_arm_outer, d_arm_inner, **kwargs):
        I = np.pi/64 * (d_arm_outer**4 - d_arm_inner**4)
        sigma_torsion_bending = torque_max * (d_arm_outer / 2) / I
        return sigma_torsion_bending

    def get_mass_arm(self, d_arm_outer, d_arm_inner, l_arm, density_arm, **kwargs):
        A = np.pi * (d_arm_outer**2 - d_arm_inner**2) / 4
        V = A * l_arm
        mass_arm = V * density_arm
        return mass_arm
    

class Arms(Functions):
    def __init__(self, material_properties, geometrical_properties, bark_properties, x0_arms, bounds_arms):
        self.mat = material_properties
        self.geo = geometrical_properties
        self.bark = bark_properties
        self.keys = x0_arms # [l_arm, d_arm_outer, d_arm_inner, alpha_arm, beta_arm, n_prop]
        self.x0 = np.array([self.geo[k] for k in self.keys])
        self.bounds = bounds_arms

        self.initial_area = np.pi * (self.geo['d_arm_outer']**2 - self.geo['d_arm_inner']**2) / 4
        self.initial_mass = self.get_mass(**self.geo, **self.mat)[0]

    def unpack(self, x):
        return dict(zip(self.keys, x))

    def objective(self, x):
        xdict = self.unpack(x)
        mass = xdict["n_prop"]*self.get_mass_arm(**xdict, **self.mat)
        return mass

    def constraint_normal_stress(self, x): # contraint 1
        xdict = self.unpack(x)
        F_normal = self.get_max_normal_force(**xdict, **self.bark, sf_forces_arm=self.geo["sf_forces_arm"])
        sigma_normal = self.get_max_normal_stress(F_normal, **xdict)
        M_bending = self.get_max_bending_moment(F_normal, **xdict)
        sigma_bending = self.get_max_bending_stress(M_bending, **xdict)
        sigma_max = sigma_normal + sigma_bending
        return self.mat["sigma_yield_arm"] - sigma_max

    def constraint_buckling(self, x): # contraint 2
        xdict = self.unpack(x)
        I = np.pi * (xdict["d_arm_outer"]**4 - xdict["d_arm_inner"]**4) / 64
        F_allowable = 0.25*np.pi**2 * self.mat["E_arm"] * I/ xdict["l_arm"]**2
        F_normal = self.get_max_normal_force(**xdict, **self.bark, sf_forces_arm=self.geo["sf_forces_arm"])
        return F_allowable - F_normal

    def constraint_shear_stress(self, x): # contraint 3
        xdict = self.unpack(x)
        F_shear = self.get_max_shear(**xdict, **self.bark, sf_forces_arm=self.geo["sf_forces_arm"])
        tau_bending = self.get_max_shear_stress_bending(F_shear, **xdict)
        T_torsion = self.get_torsion_moment(F_shear, **xdict)
        tau_torsion = self.get_max_shear_stress_torsion(T_torsion, **xdict)
        tau_max = tau_bending + tau_torsion
        return self.mat["sigma_yield_arm"] - tau_max


    def constraint_body_clearance(self, x): # contraint 4
        xdict = self.unpack(x)
        z_arm = xdict["l_arm"] * np.sin(np.radians(xdict["beta_arm"]))
        return z_arm - self.geo["min_clearance"]

    def constraint_arm_thickness(self, x): # contraint 5
        xdict = self.unpack(x)
        t = xdict["d_arm_outer"] - xdict["d_arm_inner"]
        return t - self.geo["min_thickness"]

    def constraint_propeller_clearance_y(self, x): # contraint 6
        xdict = self.unpack(x)
        prop_radius = self.geo["d_prop"] / 2
        min_distance = prop_radius*(1 + self.geo["c_prop_v"])
        y_arm = xdict["l_arm"] * np.sin(np.radians(xdict["alpha_arm"]))*np.cos(np.radians(xdict["beta_arm"]))
        return y_arm - min_distance

    def constraint_propeller_clearance_x(self, x): # contraint 7
        xdict = self.unpack(x)
        prop_radius = self.geo["d_prop"] / 2
        min_distance = prop_radius*(1 + self.geo["c_prop_h"])
        x_arm = xdict["l_arm"] * np.cos(np.radians(xdict["alpha_arm"]))*np.cos(np.radians(xdict["beta_arm"]))
        return x_arm - min_distance

    def constraint_deflection(self, x): # contraint 8
        xdict = self.unpack(x)
        max_angle_deflection = np.arctan([np.sqrt(1-(1-self.geo["thrust_loss"])**2)/(1-self.geo["thrust_loss"])])
        F_shear = self.get_max_shear(**xdict, **self.bark, sf_forces_arm=self.geo["sf_forces_arm"])
        I = np.pi * (xdict["d_arm_outer"]**4 - xdict["d_arm_inner"]**4) / 64
        theta = F_shear*xdict["l_arm"]**2 /(2*self.mat["E_arm"]*I)
        return max_angle_deflection - theta  # Ensure deflection is within limits

    def constraint_torsion(self, x): # contraint 9
        xdict = self.unpack(x)
        torque_max = self.get_propeller_torque(**self.geo)
        sigma_torsion_bending = self.get_propeller_torque_stress(torque_max, xdict["d_arm_outer"], xdict["d_arm_inner"])
        return self.mat["sigma_yield_arm"] - sigma_torsion_bending

    def constraint_principal_stress(self, x): # contraint 10
        xdict = self.unpack(x)
        F_normal = self.get_max_normal_force(**xdict, **self.bark, sf_forces_arm=self.geo["sf_forces_arm"])
        sigma_normal = self.get_max_normal_stress(F_normal, **xdict)
        M_bending = self.get_max_bending_moment(F_normal, **xdict)
        sigma_bending = self.get_max_bending_stress(M_bending, **xdict)
        torque_max = self.get_propeller_torque(**self.geo)
        sigma_torsion_bending = self.get_propeller_torque_stress(torque_max, xdict["d_arm_outer"], xdict["d_arm_inner"])
        sigma_max = sigma_normal + sigma_bending + sigma_torsion_bending
        F_shear = self.get_max_shear(**xdict, **self.bark, sf_forces_arm=self.geo["sf_forces_arm"])
        tau_bending = self.get_max_shear_stress_bending(F_shear, **xdict)
        T_torsion = self.get_torsion_moment(F_shear, **xdict)
        tau_torsion = self.get_max_shear_stress_torsion(T_torsion, **xdict)
        tau_max = tau_bending + tau_torsion
        # Calculate Von Mises stress
        sigma_principal = sigma_max/2 + np.sqrt((sigma_max/2)**2 + tau_max**2)
        return self.mat["sigma_yield_arm"] - sigma_principal
    
    def get_constraints(self):
        return [
            {'type': 'ineq', 'fun': self.constraint_normal_stress},
            {'type': 'ineq', 'fun': self.constraint_buckling},
            {'type': 'ineq', 'fun': self.constraint_shear_stress},
            {'type': 'ineq', 'fun': self.constraint_body_clearance},
            {'type': 'ineq', 'fun': self.constraint_arm_thickness},
            {'type': 'ineq', 'fun': self.constraint_propeller_clearance_y},
            {'type': 'ineq', 'fun': self.constraint_propeller_clearance_x},
            {'type': 'ineq', 'fun': self.constraint_deflection},
            {'type': 'ineq', 'fun': self.constraint_torsion},
            {'type': 'ineq', 'fun': self.constraint_principal_stress}
        ]
    
    def optimise(self):
        result = minimize(self.objective, self.x0, method='SLSQP', bounds=self.bounds, constraints=self.get_constraints())
        if result.success:
            print("Arms optimization successful.")
            for i, k in enumerate(self.keys):
                self.geo[k] = result.x[i]
        return result

class Geometry(Functions):
    def __init__(self, material_properties, geometrical_properties, bark_properties, x0_geometry, bounds_geometry):
        self.mat = material_properties
        self.geo = geometrical_properties
        self.bark = bark_properties
        self.keys = x0_geometry
        self.x0 = np.array([self.geo[k] for k in self.keys])
        self.bounds = bounds_geometry

        self.initial_area = self.get_area(**self.geo)
        self.initial_mass = self.get_mass(**self.geo, **self.mat)[0]

    def unpack(self, x):
        return dict(zip(self.keys, x))

    def objective(self, x):
        xdict = self.unpack(x)
        area = self.get_area(**xdict)
        mass = self.get_mass(**xdict, **self.mat, d_spine=self.geo["d_spine"], d_bumper=self.geo["d_bumper"], n_spine=self.geo["n_spine"], n_bumper=self.geo["n_bumper"])[0]
        w_area = 0.5
        w_mass = 1 - w_area
        return w_area * area / self.initial_area + w_mass * mass / self.initial_mass

    def constraint_height_spine(self, x): #contraint 1
        xdict = self.unpack(x)
        height = xdict['l_spine'] * np.cos(np.radians(xdict['alpha_spine'])) * np.cos(np.radians(xdict['beta_spine']))
        min_height = self.geo["d_prop"]*(0.5 + self.geo["c_prop_v_outer"]) + \
            self.geo["l_arm"] * np.cos(np.radians(self.geo["alpha_arm"])) * np.cos(np.radians(self.geo["beta_arm"]))
        return height - min_height
    
    def constraint_height_bumper(self, x): #contraint 2
        xdict = self.unpack(x)
        height = xdict['l_bumper'] * np.cos(np.radians(xdict['alpha_bumper'])) * np.cos(np.radians(xdict['beta_bumper']))
        min_height = self.geo["d_prop"]*(0.5 + self.geo["c_prop_v_outer"]) + \
            self.geo["l_arm"] * np.cos(np.radians(self.geo["alpha_arm"])) * np.cos(np.radians(self.geo["beta_arm"]))
        return height - min_height

    def constraint_force_limit(self, x): # contraint 3
        xdict = self.unpack(x)
        Fs = self.get_Fs(self.bark['W'], xdict['n_hook'], self.geo['sf_forces_spumper'])
        Fn = self.get_Fn(**xdict, W=self.bark['W'], sf_forces_spumper=self.geo['sf_forces_spumper'])
        F_tot = np.sqrt(Fs**2 + Fn**2)
        F_max = self.get_Fmax(**self.mat, **self.geo, **self.bark)
        return F_max - F_tot

    def constraint_hook_yield(self, x): # contraint 4
        xdict = self.unpack(x)
        Fs = self.get_Fs(self.bark['W'], xdict['n_hook'], self.geo['sf_forces_spumper'])
        Fn = self.get_Fn(**xdict, W=self.bark['W'], sf_forces_spumper=self.geo['sf_forces_spumper'])
        F_tot = np.sqrt(Fs**2 + Fn**2)
        Smax = self.get_Smax(F_tot, self.geo['l_hook'], self.geo['d_hook'])
        return self.mat['sigma_yield_hook'] - Smax
    
    def constraint_adhesion_angle(self, x): # contraint 5
        xdict = self.unpack(x)
        Fs = self.get_Fs(self.bark['W'], xdict['n_hook'], self.geo['sf_forces_spumper'])
        Fn = self.get_Fn(**xdict, W=self.bark['W'], sf_forces_spumper=self.geo['sf_forces_spumper'])
        return np.radians(self.geo['alpha']) + Fn/(Fs + 1e-8)

    def constraint_beta_alignment(self, x): # contraint 6
        xdict = self.unpack(x)
        return xdict['l_spine'] * np.sin(np.radians(xdict['beta_spine'])) - \
               xdict['l_bumper'] * np.sin(np.radians(xdict['beta_bumper']))
    
    def constraint_spine_spacing(self, x): # contraint 9
        xdict = self.unpack(x)
        spacing = 2 * xdict['l_spine'] * np.sin(np.radians(xdict['alpha_spine'])) * np.cos(np.radians(xdict['beta_spine']))
        return spacing - self.geo['spacing_spine']
    
    def constraint_bumper_spacing(self, x): # contraint 10
        xdict = self.unpack(x)
        spacing = 2 * xdict['l_bumper'] * np.sin(np.radians(xdict['alpha_bumper'])) * np.cos(np.radians(xdict['beta_bumper']))
        return spacing - self.geo['spacing_bumper']

    def get_constraints(self):
        return [
            {'type': 'ineq', 'fun': self.constraint_height_spine},
            {'type': 'ineq', 'fun': self.constraint_height_bumper},
            {'type': 'ineq', 'fun': self.constraint_force_limit},
            {'type': 'ineq', 'fun': self.constraint_hook_yield},
            {'type': 'ineq', 'fun': self.constraint_adhesion_angle},
            {'type': 'eq', 'fun': self.constraint_beta_alignment},
            {'type': 'eq', 'fun': self.constraint_spine_spacing},
            {'type': 'eq', 'fun': self.constraint_bumper_spacing}
        ]

    def optimise(self):
        result = minimize(self.objective, self.x0, method='SLSQP', bounds=self.bounds, constraints=self.get_constraints())
        if result.success:
            print("Geometry optimization successful.")
            for i, k in enumerate(self.keys):
                self.geo[k] = result.x[i]
        return result

class Diameter(Functions):
    def __init__(self, material_properties, geometrical_properties, bark_properties, x0_diameter, bounds_diameter):
        self.mat = material_properties
        self.geo = geometrical_properties
        self.bark = bark_properties
        self.keys = x0_diameter
        self.x0 = np.array([self.geo[k] for k in self.keys])
        self.bounds = bounds_diameter

        self.initial_area = np.pi * (self.geo['d_spine']**2 + self.geo['d_bumper']**2) / 4
        self.initial_mass = self.get_mass(**self.geo, **self.mat)[0]

    def unpack(self, x):
        return dict(zip(self.keys, x))

    def objective(self, x):
        xdict = self.unpack(x)
        area = self.get_area(**self.geo)
        mass = self.get_mass(d_spine=xdict["d_spine"], d_bumper=xdict["d_bumper"], l_spine=self.geo["l_spine"], l_bumper=self.geo["l_bumper"], n_spine=self.geo["n_spine"], n_bumper=self.geo["n_bumper"], n_hook=self.geo["n_hook"], **self.mat)[0]
        w_area = 0.5
        w_mass = 1 - w_area
        return w_area * area / self.initial_area + w_mass * mass / self.initial_mass
    
        # Stress in the spine must be less than the yield strength of the spine material
    def constraint_yield_spine(self, x): # contraint 1
        xdict = self.unpack(x)
        Fs = self.get_Fs(self.bark['W'], self.geo['n_hook'], self.geo['sf_forces_spumper'])
        Fn = self.get_Fn(**self.geo, W=self.bark['W'])
        F_tot = np.sqrt(Fs**2 + Fn**2)*self.geo['n_spine']
        S = F_tot/(np.pi * (xdict["d_spine"]/2)**2)
        return self.mat["sigma_yield_spine"] - S

    def constraint_buckling_bumper(self, x): # contraint 2
        xdict = self.unpack(x)
        amoi_bumper = np.pi/4*(xdict["d_bumper"]/2)**4 
        Fbuck = 2.04*np.pi**2*self.mat["E_bumper"]*amoi_bumper/(self.geo["l_bumper"]**2)
        return Fbuck - self.bark["W"]*self.geo["n_load"]

    def constraint_deflection_spine(self, x): # contraint 3
        xdict = self.unpack(x)
        m_hooks = 4*self.geo["n_hook"]*self.mat["m_hook"]
        m_hooks_per_spine = m_hooks / self.geo["n_spine"]
        amoi_spine = np.pi/4*(xdict["d_spine"]/2)**4 
        deflection = m_hooks_per_spine * 9.81 * self.geo["l_spine"]**3 / (3 * self.mat["E_spine"] * amoi_spine)
        max_deflection = 0.1* self.geo["l_spine"]  # Maximum deflection allowed is 10% of the spine length
        return max_deflection - deflection
    
    def get_constraints(self):
        return [
            {'type': 'ineq', 'fun': self.constraint_yield_spine},
            {'type': 'ineq', 'fun': self.constraint_buckling_bumper},
            {'type': 'ineq', 'fun': self.constraint_deflection_spine}
        ]

    def optimise(self):
        result = minimize(self.objective, self.x0, method='SLSQP', bounds=self.bounds, constraints=self.get_constraints())
        if result.success:
            print("Diameter optimization successful.")
            for i, k in enumerate(self.keys):
                self.geo[k] = result.x[i]
        return result