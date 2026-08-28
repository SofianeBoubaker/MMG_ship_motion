from solver import solver
from readFile import readFile
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz, solve_ivp

# Clear console (equivalent to 'clc' in MATLAB)
print("\033c", end="")

# --- User Inputs ---
motion = int(input("Motion test (0: turning / 1: zigzag): "))
filename = input('Input file name (e.g., filename.txt): ')
data = readFile(filename)  # Replace with actual file reading logic
data['dtR'] = float(input('Rudder execution time step (s): '))
print('Info: Rudder angle progresses in steps, increasing by dtR * delta_speed each dtR time.')
data['T_final'] = float(input('Simulation time (s): '))
data['delta_max'] = float(input('Targeted rudder angle (deg): '))
data['rho'] = 1025  # Water density (kg/m^3)

# --- Simulation Run ---
t, y, delta = solver(data, motion)

# --- Post-Processing ---
beta = np.arctan(-y[:, 1] / y[:, 0]) * 180 / np.pi  # Drift angle
psi = cumtrapz(y[:, 2], t, initial=0)  # Heading angle
U = np.sqrt(y[:, 0]**2 + y[:, 1]**2)  # Resultant speed

# Ship-fixed velocities at midship in Earth-fixed frame
x_dot = y[:, 0] * np.cos(psi) - y[:, 1] * np.sin(psi)
y_dot = y[:, 0] * np.sin(psi) + y[:, 1] * np.cos(psi)

# Earth-fixed positions at midship
x_pos = cumtrapz(x_dot, t, initial=0)
y_pos = cumtrapz(y_dot, t, initial=0)

# --- Turning Test Analysis ---
if motion == 0:
    i = 0
    check = 0  # Flag to track progress through the test
    while i < len(psi):
        if abs(psi[i]) >= np.pi / 2 and check == 0:
            check = 1
            ind_90 = i
            x_90 = abs(x_pos[ind_90])
            y_90 = abs(y_pos[ind_90])
            print(f'Advance = {x_90} m')
            print(f'Transfer = {y_90} m')

        if abs(psi[i]) >= np.pi and check == 1:
            ind_180 = i
            check = 2
            y_180 = abs(y_pos[ind_180])
            print(f'Tactical diameter = {y_180} m')

        if abs(psi[i]) >= 3 * np.pi and check == 2:
            ind_540 = i
            check = 3
            y_540 = abs(y_pos[ind_540])

        if abs(psi[i]) >= 4 * np.pi and check == 3:
            ind_720 = i
            check = 4
            y_720 = abs(y_pos[ind_720])
            print(f'Steady turning diameter = {y_540 - y_720} m')
            print(f'Steady yaw rate = {y[-1, 2] * 180 / np.pi} deg/s')
            print(f'Steady turning speed = {U[-1]} m/s')

        i += 1
        if check == 4:
            break

# --- Zig-Zag Test Analysis ---
else:
    i = 0
    check = 0
    while i < len(psi) and check < 2:
        if data['delta_max'] * y[i, 2] < 0 and check == 0:
            OSA_1 = abs(abs(psi[i] * 180 / np.pi) - abs(data['delta_max']))
            check = 1
            print(f'1st Overshoot Angle = {OSA_1} deg')

        if data['delta_max'] * y[i, 2] > 0 and check == 1:
            OSA_2 = abs(abs(psi[i] * 180 / np.pi) - abs(data['delta_max']))
            check = 2
            print(f'2nd Overshoot Angle = {OSA_2} deg')

        i += 1

# --- Plots at Midship ---
fig = 1
plt.figure(fig)
plt.subplot(2, 2, 1)
plt.plot(y_pos, x_pos)
plt.title('Midship Trajectory')
plt.xlabel('y [m]')
plt.ylabel('x [m]')
plt.grid(True)

plt.subplot(2, 2, 2)
plt.plot(t, U)
plt.title('Midship Speed U')
plt.xlabel('Time [s]')
plt.ylabel('U [m/s]')
plt.grid(True)

plt.subplot(2, 2, 3)
plt.plot(t, beta)
plt.title('Midship Drift Angle')
plt.xlabel('Time [s]')
plt.ylabel('Beta [deg]')
plt.grid(True)

plt.subplot(2, 2, 4)
plt.plot(t, y[:, 2] * 180 / np.pi)
plt.title('Yaw')
plt.xlabel('Time [s]')
plt.ylabel('Yaw Rate [deg/s]')
plt.grid(True)
plt.tight_layout()
plt.show()

fig += 1
plt.figure(fig)
plt.plot(t, np.degrees(psi), 'b', label='Heading Angle')
plt.plot(t, delta, 'r--', label='Rudder Angle')
plt.title('Yaw Angle')
plt.xlabel('Time [s]')
plt.ylabel('Yaw/Rudder Angle [deg]')
plt.grid(True)
plt.legend()

fig += 1
plt.figure(fig)
plt.plot(y_pos / data['L'], x_pos / data['L'])
plt.title('Trajectory at Midship')
plt.xlabel('y/L [-]')
plt.ylabel('x/L [-]')
plt.grid(True)
plt.axis('equal')

plt.show()