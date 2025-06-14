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

def foot_angles(var, x_bumper, y_bumper, x_foot_attachment, y_foot_attachment, l_spine): #returns alpha and beta for foot
    # Define the constraints for the optimization problem
    func = [-x_bumper*np.cos(np.deg2rad(-var[0])) - y_bumper*(np.sin(np.deg2rad(-var[0]))) + (-x_foot_attachment-l_spine*np.cos(np.deg2rad(-var[1])))*np.cos(np.deg2rad(-var[0])) + (x_foot_attachment+l_spine*np.sin(np.deg2rad(-var[1])))*np.sin(np.deg2rad(-var[0])),
            (x_foot_attachment+l_spine*np.cos(np.deg2rad(-var[1]))*np.sin(np.deg2rad(-var[0]))) + (x_foot_attachment+l_spine*np.sin(np.deg2rad(-var[1])))*np.cos(np.deg2rad(-var[0]))]

    root = fsolve(func, [0, 0]) #x is the variables for solving - initial guess
    print(f"Root found: {root}")
    return var[0], var[1]