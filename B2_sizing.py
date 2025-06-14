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

def b2_angles(x, x_b1_end, y_b1_end, x_b2_attachment, y_b2_attachment, l_hook):
    # Define the constraints for the optimization problem
    func = [-x_b1_end*np.cos(np.deg2rad(-x[0])) - y_b1_end*(np.sin(np.deg2rad(-x[0]))) + (-x_b2_attachment-l_hook*np.cos(np.deg2rad(-x[1])))*np.cos(np.deg2rad(-x[0])) + (x_b2_attachment+l_hook*np.sin(np.deg2rad(-x[1])))*np.sin(np.deg2rad(-x[0])),
            (x_b2_attachment+l_hook*np.cos(np.deg2rad(-x[1]))*np.sin(np.deg2rad(-x[0]))) + (x_b2_attachment+l_hook*np.sin(np.deg2rad(-x[1])))*np.cos(np.deg2rad(-x[0]))]

    root = fsolve(func, [0, 0])
    print(f"Root found: {root}")