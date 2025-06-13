import numpy as np
from scipy.optimize import fsolve

x_b1 = -0.195
y_b1 = -0.035

x_b2 = 0.052
y_b2 = 0.03

l_hook = 0.17 #To make the rotation equal to l_hook, enables better insertion into sleeve

#Need rotation matrices
#x' = x*cos(theta) + y*sin(theta)
#y' = -x*sin(theta) + y*cos(theta)

def func(x):
    # Define the constraints for the optimization problem
    return [-x_b1*np.cos(np.deg2rad(-x[0])) - y_b1*(np.sin(np.deg2rad(-x[0]))) + (-x_b2-l_hook*np.cos(np.deg2rad(-x[1])))*np.cos(np.deg2rad(-x[0])) + (x_b2+l_hook*np.sin(np.deg2rad(-x[1])))*np.sin(np.deg2rad(-x[0])),
            (x_b2+l_hook*np.cos(np.deg2rad(-x[1]))*np.sin(np.deg2rad(-x[0]))) + (x_b2+l_hook*np.sin(np.deg2rad(-x[1])))*
    np.cos(np.deg2rad(-x[0]))]

root = fsolve(func, [0, 0])
print(f"Root found: {root}")