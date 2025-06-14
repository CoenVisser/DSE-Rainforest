import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Initial conditions
xcg, ycg = 0, 0
thetacg = -20 * np.pi / 180
vxcg, vycg, omegacg = 1, 0, 0

thetacg0 = thetacg
lbump = 0.2
m = 2.2
I = 1
x_wall = 8
TW = 1.01
Fx = 0
Fy = m * 9.81 * (TW - 1)
M = 0
dt = 0.01

def get_bumper_position(xcg, ycg, thetacg, lbump):
    return xcg + lbump * np.cos(thetacg), ycg + lbump * np.sin(thetacg)

def update_velocity(vxcg, vycg, omegacg, axcg, aycg, alphacg, dt):
    return vxcg + axcg * dt, vycg + aycg * dt, omegacg + alphacg * dt

def update_position(xcg, ycg, thetacg, vxcg, vycg, omegacg, dt):
    return xcg + vxcg * dt, ycg + vycg * dt, thetacg + omegacg * dt

# Time and state history
ts = [0]
xcg_values, ycg_values = [xcg], [ycg]
xbumper_values, ybumper_values = [], []

for step in range(1000):
    xbumper, ybumper = get_bumper_position(xcg, ycg, thetacg, lbump)

    # Collision check
    if xbumper > x_wall:
        # Assume elastic impact and large wall reaction force
        # Estimate normal force as impulse: F * dt = -m * v_bumper_x
        v_bumper_x = vxcg + omegacg * (-lbump * np.sin(thetacg))
        F_impact = -m * v_bumper_x / dt
        Fx = F_impact
        Fy = 0

        # Torque from bumper force about CG
        M = -lbump * np.sin(thetacg) * F_impact
    else:
        Fx = m * 9.81 * (TW - 1) * np.cos(thetacg - thetacg0)
        Fy = m * 9.81 * (TW - 1) * np.sin(thetacg - thetacg0)
        M = 0

    axcg, aycg = Fx / m, Fy / m
    alphacg = M / I

    vxcg, vycg, omegacg = update_velocity(vxcg, vycg, omegacg, axcg, aycg, alphacg, dt)
    xcg, ycg, thetacg = update_position(xcg, ycg, thetacg, vxcg, vycg, omegacg, dt)

    ts.append(ts[-1] + dt)
    xcg_values.append(xcg)
    ycg_values.append(ycg)
    xbumper_values.append(xbumper)
    ybumper_values.append(ybumper)

# Plotting
x1, y1 = xcg_values, ycg_values
x2, y2 = xbumper_values, ybumper_values
t = ts

fig, ax = plt.subplots()
ax.set_aspect('equal')
line, = ax.plot([], [], 'bo-')
time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)

ax.set_xlim(min(x1 + x2) - 1, max(x1 + x2) + 1)
ax.set_ylim(min(y1 + y2) - 1, max(y1 + y2) + 1)
ax.set_title('CG and Bumper Motion with Impact')

def init():
    line.set_data([], [])
    time_text.set_text('')
    return line, time_text

def update(i):
    line.set_data([x1[i], x2[i]], [y1[i], y2[i]])
    time_text.set_text(f'Time = {t[i]:.2f} s')
    return line, time_text

ani = FuncAnimation(fig, update, frames=len(t), init_func=init, blit=True, interval=1)
plt.show()
