import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rotations import Quaternion, skew_symmetric

# Load data
with open('data/pt1_data.pkl', 'rb') as file:
    data = pickle.load(file)

gt = data['gt']
imu_f = data['imu_f']
imu_w = data['imu_w']
gnss = data['gnss']
lidar = data['lidar']

# Known extrinsic calibration
C_li = np.array([
   [0.99376, -0.09722, 0.05466],
   [0.09971, 0.99401, -0.04475],
   [-0.04998, 0.04992, 0.9975]
])
t_i_li = np.array([0.5, 0.1, 0.5])

# Transform lidar data
lidar.data = (C_li @ lidar.data.T).T + t_i_li

# Sensor variances
var_imu_f = 0.10
var_imu_w = 0.25
var_gnss = 0.01
var_lidar = 1.00

# Constants
g = np.array([0, 0, -9.81])  # gravity
l_jac = np.zeros((9, 6))
l_jac[3:, :] = np.eye(6)  # Motion model noise Jacobian
h_jac = np.zeros((3, 9))
h_jac[:, :3] = np.eye(3)  # Measurement model Jacobian

# Initial values
n = imu_f.data.shape[0]
p_est = np.zeros((n, 3))  # Position estimates
v_est = np.zeros((n, 3))  # Velocity estimates
q_est = np.zeros((n, 4))  # Orientation estimates as quaternions
p_cov = np.zeros((n, 9, 9))  # Covariance matrices

# Set initial values
p_est[0] = gt.p[0]
v_est[0] = gt.v[0]
q_est[0] = Quaternion(euler=gt.r[0]).to_numpy()
p_cov[0] = np.zeros((9, 9))

# Indices
gnss_i = 0
lidar_i = 0

def measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check):
    r_cov = np.eye(3) * sensor_var
    k_gain = p_cov_check @ h_jac.T @ np.linalg.inv(h_jac @ p_cov_check @ h_jac.T + r_cov)
    x_error = k_gain @ (y_k - p_check)
    p_hat = p_check + x_error[:3]
    v_hat = v_check + x_error[3:6]
    q_hat = Quaternion(euler=x_error[6:9]).quat_mult_left(q_check)
    p_cov_hat = (np.eye(9) - k_gain @ h_jac) @ p_cov_check
    return p_hat, v_hat, q_hat, p_cov_hat

# Main loop
for k in range(1, n):
    delta_t = imu_f.t[k] - imu_f.t[k - 1]

    # Update state with IMU inputs
    C_ns = Quaternion(*q_est[k-1]).to_mat()
    C_ns_d_f_km = C_ns @ imu_f.data[k-1]
    
    p_est[k] = p_est[k-1] + delta_t * v_est[k-1] + (delta_t**2) / 2 * (C_ns_d_f_km + g)
    v_est[k] = v_est[k-1] + delta_t * (C_ns_d_f_km + g)
    q_fr_w = Quaternion(axis_angle=imu_w.data[k-1] * delta_t)
    q_est[k] = q_fr_w.quat_mult_right(q_est[k-1])
    
    # Linearize motion model and compute Jacobians
    f_ja_km = np.eye(9)
    f_ja_km[0:3, 3:6] = np.eye(3) * delta_t
    f_ja_km[3:6, 6:9] = -skew_symmetric(C_ns_d_f_km) * delta_t
    
    # Propagate uncertainty
    q_cov_km = np.zeros((6, 6))
    q_cov_km[0:3, 0:3] = delta_t**2 * np.eye(3) * var_imu_f
    q_cov_km[3:6, 3:6] = delta_t**2 * np.eye(3) * var_imu_w
    p_cov[k] = f_ja_km @ p_cov[k-1] @ f_ja_km.T + l_jac @ q_cov_km @ l_jac.T
    
    # Check GNSS and LIDAR measurements
    if gnss_i < gnss.data.shape[0] and imu_f.t[k] == gnss.t[gnss_i]:
        p_est[k], v_est[k], q_est[k], p_cov[k] = measurement_update(var_gnss, p_cov[k], gnss.data[gnss_i].T, p_est[k], v_est[k], q_est[k])
        gnss_i += 1
        
    if lidar_i < lidar.t.shape[0] and lidar.t[lidar_i] == imu_f.t[k-1]:
        p_est[k], v_est[k], q_est[k], p_cov[k] = measurement_update(var_lidar, p_cov[k], lidar.data[lidar_i].T, p_est[k], v_est[k], q_est[k])
        lidar_i += 1

# Plot results
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(p_est[:,0], p_est[:,1], p_est[:,2], label='Estimated')
ax.plot(gt.p[:,0], gt.p[:,1], gt.p[:,2], label='Ground Truth')
ax.set_xlabel('Easting [m]')
ax.set_ylabel('Northing [m]')
ax.set_zlabel('Up [m]')
ax.set_title('Ground Truth and Estimated Trajectory')
ax.set_xlim(0, 200)
ax.set_ylim(0, 200)
ax.set_zlim(-2, 2)
ax.set_xticks([0, 50, 100, 150, 200])
ax.set_yticks([0, 50, 100, 150, 200])
ax.set_zticks([-2, -1, 0, 1, 2])
ax.legend(loc=(0.62, 0.77))
ax.view_init(elev=45, azim=-50)
plt.show()
