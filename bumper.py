import numpy as np
def get_bumper_properties(l_prop, beta_prop, h_prop, d_prop):
    beta_bumper = np.arctan2(l_prop*np.sin(np.deg2rad(beta_prop))+0.5*h_prop, l_prop*np.cos(np.deg2rad(beta_prop))+0.5*d_prop)
    l_bumper = l_prop*np.cos(np.deg2rad(beta_prop))/np.cos(beta_bumper)+0.5*d_prop/np.sin(beta_bumper)
    beta_bumper = np.rad2deg(beta_bumper)
    return beta_bumper, l_bumper