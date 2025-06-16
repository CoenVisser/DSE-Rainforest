import numpy as np

def get_bumper_properties(l_cg, h_platform, l_arm, beta_arm, h_prop, d_prop, c_prop_v_outer, **kwargs):
    x_bumper_prime = l_arm*np.cos(np.deg2rad(beta_arm))+(0.5+c_prop_v_outer)*d_prop
    y_bumper_prime = 0
    z_bumper_prime = l_arm*np.sin(np.deg2rad(beta_arm))+0.5*h_prop
    beta_bumper = np.arctan2(z_bumper_prime, x_bumper_prime)
    l_bumper = np.sqrt(x_bumper_prime**2 + y_bumper_prime**2 + z_bumper_prime**2)
    beta_bumper = np.rad2deg(beta_bumper)
    x_bumper = x_bumper_prime
    y_bumper = y_bumper_prime
    z_bumper = -z_bumper_prime - (h_platform - l_cg)
    return l_bumper, beta_bumper, x_bumper, y_bumper, z_bumper

def get_foot_properties(l_cg, l_foot, x_bumper, z_bumper, spacing_foot, l_platform, h_platform, **kwargs):
    A = x_bumper**2 + z_bumper**2
    B = -2*l_foot**2*x_bumper
    C = l_foot**4 - l_foot**2*z_bumper**2

    x_foot = (-B + np.sqrt(B**2 - 4*A*C)) / (2*A)
    assert x_foot >= 0, "Calculated x_foot is negative, check inputs."

    z_foot = (l_foot**2 - x_bumper*x_foot) / z_bumper
    assert z_foot >= 0, "Calculated z_foot is negative, check inputs."

    y_foot = spacing_foot / 2

    x_foot_prime = x_foot
    y_foot_prime = y_foot
    z_foot_prime = z_foot - l_cg

    beta = np.rad2deg(np.arcsin(z_foot_prime / l_foot))
    alpha = np.rad2deg(np.arctan2(y_foot_prime, x_foot_prime))

    x_corner = l_platform / 2
    z_corner = h_platform / 2

    check = (x_foot - x_bumper) * (z_corner - z_bumper) - (z_foot - z_bumper) * (x_corner - x_bumper)
    assert check >= 0, "BARK platform interferes, check inputs."

    return alpha, beta, x_foot, y_foot, z_foot

def get_Fs(W, n_hook, sf_forces_spumper, **kwargs):
    return W / n_hook  * sf_forces_spumper

def get_Fn(l_cg, l_spine, alpha_spine, beta_spine, l_foot, alpha_foot, beta_foot, W, n_hook, sf_forces_spumper, **kwargs):
    Fn = - W*(l_cg + l_spine*np.cos(np.radians(alpha_spine))*np.sin(np.radians(beta_spine))) / (
        l_spine*np.cos(np.radians(alpha_spine))*np.cos(np.radians(beta_spine)) +
        l_foot*np.cos(np.radians(alpha_foot))*np.cos(np.radians(beta_foot))
    ) / n_hook
    return Fn * sf_forces_spumper

def get_spine_diameter(l_cg, l_spine, alpha_spine, beta_spine, l_foot, alpha_foot, beta_foot, W, n_hook, m_hook, sf_forces_spumper, n_spine, sigma_yield_spine, E_spine, **kwargs):
    ds = [0.005]
    
    # yield
    Fn = get_Fn(l_cg, l_spine, alpha_spine, beta_spine, l_foot, alpha_foot, beta_foot, W, n_hook, sf_forces_spumper)
    Fs = get_Fs(W, n_hook, sf_forces_spumper)
    F_tot = np.sqrt(Fs**2 + Fn**2)*n_spine
    ds.append(np.sqrt(4*F_tot / (np.pi * sigma_yield_spine)))

    # deflection
    m_hooks = 4 * n_hook * m_hook
    m_hooks_per_spine = m_hooks / n_spine
    ds.append(2*(4 * m_hooks_per_spine * 9.81 * l_spine**2 / (0.3 * E_spine * np.pi))**(1/4))
    
    return max(ds)

def get_foot_diameter(l_foot, beta_foot, m, sf_forces_spumper, impact_velocity, impact_time, E_foot, sigma_yield_foot, **kwargs):
    df = [0.005]
    
    # buckling
    F = sf_forces_spumper * m * impact_velocity/impact_time
    df.append(32*F * l_foot**2 / (2.04*np.pi**3 * E_foot))

    # deflection
    F = sf_forces_spumper * m * impact_velocity/impact_time * np.sin(np.radians(beta_foot))
    df.append(2 * (F * l_foot**2 / (3* 0.01 * E_foot * np.pi))**(1/4))

    # yield
    M = F* l_foot
    df.append(2*(4* M / (np.pi * sigma_yield_foot))**(1/3))

    return max(df)

def get_bumper_diameter(l_bumper, beta_bumper, m, sf_forces_spumper, impact_velocity, impact_time, E_bumper, sigma_yield_bumper, **kwargs):
    db = [0.005]
    
    # buckling
    F = sf_forces_spumper * m * impact_velocity/impact_time
    db.append(32*F * l_bumper**2 / (2.04*np.pi**3 * E_bumper))

    # deflection
    F = sf_forces_spumper * m * impact_velocity/impact_time * np.sin(np.radians(beta_bumper))
    db.append(2 * (F * l_bumper**2 / (3*0.01 * E_bumper * np.pi))**(1/4))

    # yield
    M = F* l_bumper
    db.append(2*(4* M / (np.pi * sigma_yield_bumper))**(1/3))

    return max(db)
