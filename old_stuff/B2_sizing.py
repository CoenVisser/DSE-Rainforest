import numpy as np
from scipy.optimize import fsolve

#x_b1 = -0.195
#y_b1 = -0.035

#x_b2 = 0.052
#y_b2 = 0.03

#l_hook = 0.17 #To make the rotation equal to l_hook, enables better insertion into sleeve

#Need rotation matrices
#x' = x*cos(theta) + y*sin(theta)
#y' = -x*sin(theta) + y*cos(theta)

def foot_angles(l_cg, l_foot, x_bumper, z_bumper, spacing_foot, l_platform, h_platform, **kwargs):
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



    


# def foot_angles_old(var, x_bumper, y_bumper, x_foot_attachment, y_foot_attachment, l_spine): #returns alpha and beta for foot
#     # Define the constraints for the optimization problem
#     func = [-x_bumper*np.cos(np.deg2rad(-var[0])) - y_bumper*(np.sin(np.deg2rad(-var[0]))) + (-x_foot_attachment-l_spine*np.cos(np.deg2rad(-var[1])))*np.cos(np.deg2rad(-var[0])) + (x_foot_attachment+l_spine*np.sin(np.deg2rad(-var[1])))*np.sin(np.deg2rad(-var[0])),
#             (x_foot_attachment+l_spine*np.cos(np.deg2rad(-var[1]))*np.sin(np.deg2rad(-var[0]))) + (x_foot_attachment+l_spine*np.sin(np.deg2rad(-var[1])))*np.cos(np.deg2rad(-var[0]))]

#     root = fsolve(func, [0, 0]) #x is the variables for solving - initial guess
#     print(f"Root found: {root}")
#     return var[0], var[1]