from MMG_3DOF import MMG_3DOF
import numpy as np
from scipy.integrate import solve_ivp
from scipy.integrate import cumtrapz

def solver(data, motion):
    """
    Solves the equation of motion y_dot = f(t, y) with f as the MMG_3DOF function.

    Inputs:
        data: Dictionary containing ship parameters and simulation settings.
        motion: 0 for turning test, 1 for zig-zag test.

    Outputs:
        t: Time vector (s).
        y: State variables [u, vm, r] over time at midship.
        delta_vec: Rudder angle history (deg).
    """

    # Input validation
    if motion not in [0, 1]:
        raise ValueError("Motion must be 0 (turning) or 1 (zig-zag).")

    # Initialization
    t = np.array([0.0])
    y = np.array([[data['U0'], 0, 0]])  # Initial speed conditions
    dtR = data['dtR']  # Rudder execution time (s)
    delta_max = data['delta_max']  # Maximum rudder deflection (deg)
    delta = data['delta0']
    delta_vec = np.array([delta])

    # Simulation loop
    while t[-1] < data['T_final']:
        # Phase 1: Ramp rudder angle to delta_max
        while abs(delta) <= abs(delta_max) and t[-1] < data['T_final']:
            # Set the time span for solving the equation of motion
            if (t[-1] + dtR) >= data['T_final']:
                tspan = np.array([t[-1], data['T_final']])
            else:
                tspan = np.array([t[-1], t[-1] + dtR])
            
            Eta = y[-1, :]  # Initial conditions of state variables
            sol = solve_ivp(MMG_3DOF,tspan,Eta,t_eval=np.linspace(tspan[0], tspan[1], 10),args=(data,delta))
            tt, yt = sol.t, sol.y.T
            # Append results
            t = np.concatenate([t, tt])  
            y = np.concatenate([y, yt])
            delta_vec = np.concatenate([delta_vec, np.full(tt.shape, delta)])

            # Update delta
            delta += np.sign(delta_max) * data['delta_speed'] * min(dtR, (data['T_final'] - t[-1]))

        # Phase 2: Motion with a constant rudder angle
        psi_current = cumtrapz(y[:, 2], t, initial=0)[-1]  # Last heading angle
        delta = delta_max

        while (motion * np.sign(delta_max) * psi_current * 180 / np.pi) < abs(delta_max) and t[-1] < data['T_final']:
            if motion == 0 or (t[-1] + dtR) >= data['T_final']:
                tspan = np.array([t[-1], data['T_final']])
            else:
                tspan = np.array([t[-1], t[-1] + dtR])

            Neval = 10 
            if motion == 0 : Neval = int(np.ceil(((tspan[1]-tspan[0])/dtR)*10)) #10 points each dTr intervale

            Eta = y[-1, :]
            sol = solve_ivp(MMG_3DOF,tspan,Eta,t_eval=np.linspace(tspan[0], tspan[1], Neval),args=(data,delta))
            tc, yc = sol.t, sol.y.T
            # Append results
            t = np.concatenate([t, tc])
            y = np.concatenate([y, yc])
            delta_vec = np.concatenate([delta_vec, np.full(tc.shape, delta_max)])

            psi_current = cumtrapz(y[:, 2], t, initial=0)[-1]

        # Prepare the next zig-zag cycle
        delta_max = -delta_max  # Flip the targeted rudder direction
        delta += np.sign(delta_max) * data['delta_speed'] * min(dtR, (data['T_final'] - t[-1]))

    return t, y, delta_vec