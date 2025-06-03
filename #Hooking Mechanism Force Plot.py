#Hooking Mechanism Force Plot
import numpy as np
import matplotlib.pyplot as plt

num = 100

#Material Properties
l = 0.005#length of the hooks
d = 0.001#diameter of the hooks
#K = np.array([[Ex_sp, Exy_sp], [Exy_sp, Ey_sp]])
n = 120 #number of hooks
alpha = 1/6*np.pi# load angle
E_m = 200*10**9 #Elastic modulus
v_m = 0.29 #Poisson's ratio
R_tip = 20*10**(-6) #Radius of curvature of the spine

#Tree Properties
mu_asp = 0.20 #Coefficient of friction
Ex_asp = 9.8 *10**9 #Young's modulus in Pa
v_asp = 0.4

#BARK properties
m = 2.7

#Calculation of Forces
Fg = m*9.81
Fs = Fg/n
Fn = Fs*alpha
F_asp = np.sqrt(Fs**2 + Fn**2)
sigma_max = 32*F_asp*l*d/(np.pi*d**4)

E_tot = (1-v_m**2)/E_m + (1-v_asp**2)/Ex_asp
f_max = (np.pi*sigma_max/(1-2*v_asp))**3 * 9*R_tip**2/(2*E_tot**2)

#Plotting
fig, ax = plt.subplots(subplot_kw={'projection':'polar'})
theta = np.linspace(-alpha, np.arctan(mu_asp), num)
r = [f_max]*num
ax.plot(theta, r)

ax.grid(True)
ax.set_title('Force Plot for Hooking Mechanism')
plt.show()