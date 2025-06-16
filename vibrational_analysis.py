import numpy as np

#Different modes
k_mode = np.array([0.596864*np.pi, 1.494418*np.pi, 2.50025*np.pi, 3.49999*np.pi, 14.13716839, 17.27875953])
E_arm = 40e9
I_arm = np.pi/64 * (0.0075**4-0.005**4)
rho_arm = 1400
A_arm = np.pi/4 * (0.0075**2-0.005**2)
L_arm = 0.1398

def vibration_prop(k, E, I, rho, A, L):
    f_prop = k**2/(2*np.pi)*np.sqrt(E*I/(rho*A*L**4))
    return f_prop

print(vibration_prop(k_mode, E_arm, I_arm, rho_arm, A_arm, L_arm))

rpm = 10000
f_actual = rpm* 2*np.pi/60
print(f_actual)
#rpm needs to be within a certain range