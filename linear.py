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