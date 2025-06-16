import numpy as np
from matplotlib import pyplot as plt

sleeve_angle = 21.8 # degrees
aperture_angle = 78.14 # degrees
cutout_angle = sleeve_angle + 90 - aperture_angle # degrees

diameter = 0.5 # cm

cutout_spacing = 1 # cm
cutout_x0 = 1 # cm

cutout_x = np.array([0, cutout_spacing, cutout_spacing - diameter/(2*np.tan(np.radians(cutout_angle)))])
cutout_y = np.array([diameter/2, 0, diameter/2])

x = np.hstack((0, cutout_x + cutout_x0, cutout_x + 2*cutout_x0, cutout_x + 3*cutout_x0, 5*cutout_x0))
y = np.hstack((diameter/2, cutout_y, cutout_y, cutout_y, diameter/2))
x_wall = [x[0], x[-1]]
y_wall = [-diameter/2, -diameter/2]

fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(x, y, 'k-', linewidth=2)
ax.plot(x_wall, y_wall, 'k-', linewidth=2)
ax.plot([x[0], x[0]], [-diameter/2, diameter/2], 'k--', linewidth=1)
ax.plot([x[-1], x[-1]], [-diameter/2, diameter/2], 'k--', linewidth=1)

ax.fill_between(x, y, color='grey', alpha=0.8)
ax.fill_between(x_wall, y_wall, color='grey', alpha=0.8)
ax.set_aspect('equal')
ax.set_xlim(x[0]-diameter, x[-1]+diameter)
ax.set_ylim(-diameter, diameter)
ax.set_xlabel('X (cm)')
ax.set_ylabel('Y (cm)')
ax.set_title('Cutout Plot')
plt.grid()
plt.show()


