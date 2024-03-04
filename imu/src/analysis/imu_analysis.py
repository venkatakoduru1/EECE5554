import pandas as pd
import matplotlib.pyplot as plt
import math
import numpy as np

def quaternion_to_euler(qx, qy, qz, qw):
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

# Load CSV file into a pandas DataFrame
csv_file_path = '/home/karthikkoduru1/Bag_text/stationary_1/imu.csv'
df = pd.read_csv(csv_file_path)

# Convert timestamp to pandas datetime object
# df['Time'] = pd.to_datetime(df['Time'])

# Adjusting timestamps to start from 0
df['Time'] -= df['Time'].iloc[0] 

# Convert angular velocity from radians to degrees
df['imu.angular_velocity.x_deg'] = np.degrees(df['imu.angular_velocity.x'])
df['imu.angular_velocity.y_deg'] = np.degrees(df['imu.angular_velocity.y'])
df['imu.angular_velocity.z_deg'] = np.degrees(df['imu.angular_velocity.z'])

# Plotting gyro data in degrees
plt.figure(figsize=(10, 6))
plt.plot(df['Time'].to_numpy(), df['imu.angular_velocity.x_deg'].to_numpy(), label='Rotational Rate X')
plt.plot(df['Time'].to_numpy(), df['imu.angular_velocity.y_deg'].to_numpy(), label='Rotational Rate Y')
plt.plot(df['Time'].to_numpy(), df['imu.angular_velocity.z_deg'].to_numpy(), label='Rotational Rate Z')
plt.title('rotational rate from Gyro in degrees/s on x,y & z')
plt.xlabel('Time')
plt.ylabel('rotational rate (degrees/s)')
plt.legend()
plt.show()


# Plotting accelerometer data
plt.figure(figsize=(10, 6))
plt.plot(df['Time'].to_numpy(), df['imu.linear_acceleration.x'].to_numpy(), label='Linear Acceleration X')
plt.plot(df['Time'].to_numpy(), df['imu.linear_acceleration.y'].to_numpy(), label='Linear Acceleration Y')
plt.plot(df['Time'].to_numpy(), df['imu.linear_acceleration.z'].to_numpy(), label='Linear Acceleration Z')
plt.title('Linear Acceleration from Accelerometer')
plt.xlabel('Time')
plt.ylabel('Linear Acceleration (m/s^2)')
plt.legend()
plt.show()

# Retrieve quaternion values from DataFrame
qx_values = df['imu.orientation.x'].to_numpy()
qy_values = df['imu.orientation.y'].to_numpy()
qz_values = df['imu.orientation.z'].to_numpy()
qw_values = df['imu.orientation.w'].to_numpy()

# Convert quaternion to Euler angles
euler_angles = []
for qx, qy, qz, qw in zip(qx_values, qy_values, qz_values, qw_values):
    euler_angles.append(quaternion_to_euler(qx, qy, qz, qw))

# Separate Euler angles into X, Y, Z lists
X_values, Y_values, Z_values = zip(*euler_angles)

# Plot Euler angles
plt.figure(figsize=(10, 6))
plt.plot(df['Time'].to_numpy(), X_values, label='Euler X')
plt.plot(df['Time'].to_numpy(), Y_values, label='Euler Y')
plt.plot(df['Time'].to_numpy(), Z_values, label='Euler Z')
plt.title('Euler Angles')
plt.xlabel('Time')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.show()

# Plot histograms of rotation in x, y, z
plt.figure(figsize=(10, 6))
plt.hist(X_values, bins=50, alpha=0.5, label='Rotation X')
plt.title('Histogram of Rotation in X')
plt.xlabel('Angle (degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.show()

# Plot histograms of rotation in x, y, z
plt.figure(figsize=(10, 6))
plt.hist(Y_values, bins=50, alpha=0.5, label='Rotation Y')
plt.title('Histogram of Rotation in Y')
plt.xlabel('Angle (degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.show()

# Plot histograms of rotation in x, y, z
plt.figure(figsize=(10, 6))
plt.hist(Z_values, bins=50, alpha=0.5, label='Rotation Z')
plt.title('Histogram of Rotation in Z')
plt.xlabel('Angle (degrees)')
plt.ylabel('Frequency')
plt.legend()
plt.show()




