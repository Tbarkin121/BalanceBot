import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import time
import torch

log_dir = 'StaticTesting/'
df = pd.read_csv(log_dir + 'alphas.csv')    
raw_alpha = df.to_numpy()

plt.figure("Alpha Station Points", figsize=(10, 10), dpi=80)
ax = plt.axes()
plt.plot(raw_alpha[:,1:]*180/torch.pi)
plt.title('Alpha Station Points')
ax.set_xlabel('Samples')
ax.set_ylabel('Values')
plt.grid(True)
plt.show()
# plt.xlim([0,100])

#%%

df = pd.read_csv(log_dir+'state_global.csv')
raw_state_global = df.to_numpy()

df = pd.read_csv(log_dir+'as.csv')
raw_air_speed = df.to_numpy()

df = pd.read_csv(log_dir+'gs.csv')
raw_ground_speed = df.to_numpy()

df = pd.read_csv(log_dir+'rk4_euler_angs.csv')
raw_euler = df.to_numpy()

df = pd.read_csv(log_dir+'rk4_lv_loc.csv')
raw_loc_vel = df.to_numpy()

df = pd.read_csv(log_dir+'rk4_av_loc.csv')
raw_loc_avel = df.to_numpy()

df = pd.read_csv(log_dir+'force.csv')
raw_loc_force = df.to_numpy()

df = pd.read_csv(log_dir+'torque.csv')
raw_loc_torque = df.to_numpy()

df = pd.read_csv(log_dir+'lift_prof.csv')
raw_lift_dist = df.to_numpy()

df = pd.read_csv(log_dir+'drag_prof.csv')
raw_drag_dist = df.to_numpy()


total_gliders = 1
curr_glider = 1

x_raw = raw_state_global[curr_glider-1::total_gliders,1]
y_raw = raw_state_global[curr_glider-1::total_gliders,2]
z_raw = raw_state_global[curr_glider-1::total_gliders,3]
vx_raw = raw_state_global[curr_glider-1::total_gliders,8]
vy_raw = raw_state_global[curr_glider-1::total_gliders,9]
vz_raw = raw_state_global[curr_glider-1::total_gliders,10]
avx_raw = raw_state_global[curr_glider-1::total_gliders,11]
avy_raw = raw_state_global[curr_glider-1::total_gliders,12]
avz_raw = raw_state_global[curr_glider-1::total_gliders,13]
roll_raw = raw_euler[curr_glider-1::total_gliders,1]
pitch_raw = raw_euler[curr_glider-1::total_gliders,2]
yaw_raw = raw_euler[curr_glider-1::total_gliders,3]
gs_raw = raw_ground_speed[curr_glider-1::total_gliders,1]
as_raw = raw_air_speed[curr_glider-1::total_gliders,1]
force_x_raw = raw_loc_force[curr_glider-1::total_gliders,1]
force_y_raw = raw_loc_force[curr_glider-1::total_gliders,2]
force_z_raw = raw_loc_force[curr_glider-1::total_gliders,3]
torque_x_raw = raw_loc_torque[curr_glider-1::total_gliders,1]
torque_y_raw = raw_loc_torque[curr_glider-1::total_gliders,2]
torque_z_raw = raw_loc_torque[curr_glider-1::total_gliders,3]
lift_dist_raw = raw_lift_dist[curr_glider-1::total_gliders,3]
drag_dist_raw = raw_drag_dist[curr_glider-1::total_gliders,3]

#%%

dpi_setting = 80
data_start = 1
data_end = len(force_x_raw)

roll = roll_raw[data_start:data_end] * 180.0/torch.pi
pitch = pitch_raw[data_start:data_end] * 180.0/torch.pi
yaw = yaw_raw[data_start:data_end] * 180.0/torch.pi
force_x_data = force_x_raw[data_start:data_end]
force_y_data = force_y_raw[data_start:data_end]
force_z_data = force_z_raw[data_start:data_end]
torque_x_data = torque_x_raw[data_start:data_end]
torque_y_data = torque_y_raw[data_start:data_end]
torque_z_data = torque_z_raw[data_start:data_end]





plt.figure("Force Plot", figsize=(10, 10), dpi=dpi_setting)
ax = plt.axes()
plt.plot(pitch, force_x_data)
plt.plot(pitch, force_y_data)
plt.plot(pitch, force_z_data)
plt.title('Local Force')
ax.set_xlabel('Pitch (Deg)')
ax.set_ylabel('Force (N)')
plt.grid(True)
ax.legend(['X', 'Y', 'Z'])
plt.show()

plt.figure("Torque Plot", figsize=(10, 10), dpi=dpi_setting)
ax = plt.axes()
plt.plot(pitch, torque_x_data)
plt.plot(pitch, torque_y_data)
plt.plot(pitch, torque_z_data)
plt.title('Local Torque')
ax.set_xlabel('Pitch (Deg)')
ax.set_ylabel('Torque (Nm)')
plt.grid(True)
ax.legend(['X', 'Y', 'Z'])
plt.show()


alpha_raw1 = raw_alpha[curr_glider-1::total_gliders,1]
# ang = np.reshape(raw_alpha[1:,1],(-1,1,1))
ang = alpha_raw1[1:]
# ang = alpha_raw1[2:]

c = np.cos(-ang*0)
s = np.sin(-ang*0)
R = torch.tensor([[c, -s],[s, c]])
R = torch.permute(R,(2,0,1))

force_x = torch.reshape(torch.tensor(force_x_raw[1:]), (-1,1,1))
force_z = torch.reshape(torch.tensor(force_z_raw[1:]), (-1,1,1))
force = torch.cat((force_x, force_z), 1)

force_rot = torch.matmul(R, force)

Drag = force_rot[:, 0, 0]
Lift = force_rot[:, 1, 0]

Drag = force_x[:,0,0]
Lift = force_z[:,0,0]

LoD = Lift / Drag

plt.figure("Torque Plot", figsize=(10, 10), dpi=dpi_setting)
ax = plt.axes()
plt.plot(pitch, LoD)
plt.title('Lift over Drag')
ax.set_xlabel('Pitch (Deg)')
ax.set_ylabel('Lift Over Drag')
plt.grid(True)
ax.legend(['X', 'Y', 'Z'])
plt.show()

plt.figure("Torque Plot", figsize=(10, 10), dpi=dpi_setting)
ax = plt.axes()
plt.plot(pitch, Lift)
plt.title('Lift')
ax.set_xlabel('Pitch (Deg)')
ax.set_ylabel('Lift')
plt.grid(True)
ax.legend(['X', 'Y', 'Z'])
plt.show()

plt.figure("Drag", figsize=(10, 10), dpi=dpi_setting)
ax = plt.axes()
plt.plot(pitch, Drag)
plt.title('Local Force')
ax.set_xlabel('Pitch (Deg)')
ax.set_ylabel('Drag')
plt.grid(True)
ax.legend(['X', 'Y', 'Z'])
plt.show()

#%%

drag_dist = raw_drag_dist[0, 1:21]

plt.figure("Lift Dist", figsize=(10, 10), dpi=dpi_setting)
ax = plt.axes()
for i in range(raw_lift_dist.shape[0]):
    lift_dist = raw_lift_dist[i, 1:21]
    plt.plot(np.arange(20), lift_dist)

plt.title('Lift Dist')
ax.set_xlabel('Station Pts')
ax.set_ylabel('Lift')
plt.grid(True)
plt.show()

plt.figure("Drag Dist", figsize=(10, 10), dpi=dpi_setting)
ax = plt.axes()
for i in range(raw_lift_dist.shape[0]):
    drag_dist = raw_drag_dist[i, 1:21]
    # drag_dist = raw_drag_dist[i, 21:41]
    # drag_dist = raw_drag_dist[i, 41:61]
    # drag_dist = raw_drag_dist[i, 61:81]
    plt.plot(np.arange(20), drag_dist)

plt.title('Drag Dist')
ax.set_xlabel('Station Pts')
ax.set_ylabel('Drag')
plt.grid(True)
plt.show()

#%%

print('force')
print(raw_loc_force)
print('torque')
print(raw_loc_torque)

#%%


