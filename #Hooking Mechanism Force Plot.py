#Hooking Mechanism Force Plot
import numpy as np
import matplotlib.pyplot as plt

num = 100                   #Plotting variable

#Hook Properties
l_hook = 0.02               #length of the hooks
d_hook = 0.001              #diameter of the hooks
n_hook = 120                #number of hooks
R_tip = 20* 10**(-6)         #Radius of curvature of the spine

#Hook Material Properties
E_m = 200* 10**9             #Elastic modulus
v_m = 0.29                  #Poisson's ratio
sigma_yield = 290* 10**6

#Spine Properties
l_spine = 0.2
d_spine = 0.01
n_spine = 2

#Loading Properties
alpha = 1/12*np.pi          # load angle

#Tree Properties
mu_asp = 0.20               #Coefficient of friction
Ex_asp = 9.8 *10**9         #Young's modulus in Pa
v_asp = 0.4                 #Poisson's ratio   

#BARK properties
m = 2.7                     #mass of the bark in kg   

#Calculation of Forces
Fg = m*9.81                                                 #Gravitational force
Fs = Fg/n_hook                                              #Vertical force per hook
Fn = Fs*alpha                                               #Horizontal force per hook
F_asp = np.sqrt(Fs**2 + Fn**2)                              #Force on asperity
sigma_max = 32*F_asp*l_hook*d_hook/(np.pi*d_hook**4)        #Maximum induced stress

E_tot = 1/((1-v_m**2)/E_m + (1-v_asp**2)/Ex_asp)            #Not sure about the 1/ part

f_tree = (np.pi*sigma_max/(1-2*v_asp))**3 * 9*R_tip**2/(2*E_tot**2)
f_mat = np.pi/4 * d_hook**2 * sigma_yield
f_max = min(f_tree, f_mat)                                  #Minimum sizing force

# #Plotting
# fig, ax = plt.subplots(subplot_kw={'projection':'polar'})
# theta = np.linspace(-alpha, np.arctan(mu_asp)+0.5*np.pi, num)
# r = [f_max]*num
# ax.fill_between(theta, r, color='green', alpha=0.5)
# theta_upper = [np.arctan(mu_asp)+0.5*np.pi] * num
# r_upper = np.linspace(0, f_max, num)
# ax.plot(theta_upper, r_upper, color='red')

# theta_lower = [-alpha] * num
# r_lower = np.linspace(0, f_max, num)
# ax.fill_between(theta_lower, r_lower, color='blue')

# ax.grid(True)
# ax.set_title('Force Plot for Hooking Mechanism')
# plt.show()


# Geometrical parameters

lh = 0.3
lb = 0.3

ah = 20
ab = 20

bh = 10
bb = 10

x = 0.1

Fs = m*9.81/n
Fnb = m*9.81*(x + lh*np.cos(np.deg2rad(ah))*np.sin(np.deg2rad(bh)))/(lh*np.cos(np.deg2rad(ah))*np.cos(np.deg2rad(bh)) + lb*np.cos(np.deg2rad(ab))*np.cos(np.deg2rad(bb)))/n
Fnh = -Fnb

Fmax = np.sqrt(Fs**2 + Fnh**2)
theta = np.arctan2(Fnb, Fnh)
