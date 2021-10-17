import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Import data using Pandas (install with: pip install pandas)
data = pd.read_csv("data.csv")
print(data)

# Retrieve data columns
D = data.to_numpy();
t = D[:,0]
pan = D[:,1]
tilt = D[:,2]
r_us = D[:,3] + 7 # to account for the sensor offset
r_tof = D[:,4] + 7

# Create a basic plot
plt.plot(t,r_us)
plt.plot(t,r_tof)
plt.legend(["us","tof"])
plt.ylabel("Range (mm)")
plt.xlabel("Time (s)")
plt.xlim([min(t),max(t)])
plt.savefig('basic.png') # Save plot to a file
plt.show()

# Process data -- convert to position
# Compute angle off z-axis (azimuth)
phi = (90-tilt)*np.pi/180
# Compute angle around z-axis (polar angle)
theta = pan*np.pi/180

# Convert to cartesian
x_us = r_us*np.cos(theta)*np.sin(phi)
y_us = r_us*np.sin(theta)*np.sin(phi)
z_us = r_us*np.cos(phi)

x_tof = r_tof*np.cos(theta)*np.sin(phi)
y_tof = r_tof*np.sin(theta)*np.sin(phi)
z_tof = r_tof*np.cos(phi)

# Scatter plot of 3D points
fig = plt.figure()
fig.tight_layout()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_us,y_us,z_us,s=2)
ax.scatter(x_tof,y_tof,z_tof,s=2)
plt.legend(["us","tof"])
ax.set_ylim3d(-2000, 2000)
ax.set_xlim3d(-2000, 2000)
ax.set_zlim3d(0, 2000)
plt.show()
