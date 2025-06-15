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

