import numpy as np

def MMG_3DOF(_, Z, data, delta_deg):
    """
    Computes the time derivatives of the ship's state vector [u, vm, r]
    using the MMG 3-degree-of-freedom model from Yasukawa, H. & Yoshimura, Y. (2015).
    Inputs:
        Z: State vector [u, vm, r] (surge velocity, sway velocity, yaw rate) at midship.
        data: Dictionary containing ship parameters and hydrodynamic coefficients.
        delta_deg: Rudder angle (degrees).
    Output:
        Z_dot: Time derivatives [u_dot, vm_dot, r_dot].
    """
    delta = delta_deg * np.pi / 180  # Convert rudder angle to radians

    # State variables at midship
    u = Z[0]  # Surge velocity at midship
    vm = Z[1]  # Sway velocity at midship (vm = v - data.xG * r)
    r = Z[2]  # Yaw rate at midship

    beta = np.arctan(-vm / u)  # Drift angle at midship
    U = np.sqrt(u**2 + vm**2)  # Resultant speed at midship

    # Dimensional factors
    Fdim = 0.5 * data['rho'] * data['L'] * data['d'] * U**2  # Force scaling
    Ndim = Fdim * data['L']  # Moment scaling
    Mdim = 0.5 * data['rho'] * data['L']**2 * data['d']  # Mass scaling
    Idim = Mdim * data['L']**2  # Moment of inertia scaling

    # Ship mass and inertia
    m = data['rho'] * data['CB'] * data['L'] * data['B'] * data['d']  # Ship mass estimation
    Iz = m * (0.25 * data['L'])**2  # Ship moment of inertia approximation

    # Mass matrix
    M = np.array([
        [m + data['mx_prime'] * Mdim, 0, 0],
        [0, m + data['my_prime'] * Mdim, data['xG'] * m],
        [0, data['xG'] * m, Iz + data['xG']**2 * m + data['Jz_prime'] * Idim]
    ])

    # Non-dimensional derivatives coefficients
    X0_prime = abs(data['X0_prime'])  # Convention: X0 > 0 ensures resistance opposes surge motion
    Xvv_prime = data['Xvv_prime']
    Xvvvv_prime = data['Xvvvv_prime']
    Xrr_prime = data['Xrr_prime']
    Xvr_prime = data['Xvr_prime']
    Yv_prime = data['Yv_prime']
    Yvvv_prime = data['Yvvv_prime']
    Yr_prime = data['Yr_prime']
    Yrrr_prime = data['Yrrr_prime']
    Yvrr_prime = data['Yvrr_prime']
    Yvvr_prime = data['Yvvr_prime']
    Nv_prime = data['Nv_prime']
    Nvvv_prime = data['Nvvv_prime']
    Nr_prime = data['Nr_prime']
    Nrrr_prime = data['Nrrr_prime']
    Nvrr_prime = data['Nvrr_prime']
    Nvvr_prime = data['Nvvr_prime']

    vm_prime = vm / U
    r_prime = r * data['L'] / U

    # Hull hydrodynamic derivatives at midship
    XH = Fdim * (-X0_prime + Xvv_prime * vm_prime**2 + Xvr_prime * vm_prime * r_prime +
                 Xrr_prime * r_prime**2 + Xvvvv_prime * vm_prime**4)
    YH = Fdim * (Yv_prime * vm_prime + Yr_prime * r_prime + Yvvv_prime * vm_prime**3 +
                 Yvvr_prime * vm_prime**2 * r_prime + Yvrr_prime * vm_prime * r_prime**2 +
                 Yrrr_prime * r_prime**3)
    NH = Ndim * (Nv_prime * vm_prime + Nr_prime * r_prime + Nvvv_prime * vm_prime**3 +
                 Nvvr_prime * vm_prime**2 * r_prime + Nvrr_prime * vm_prime * r_prime**2 +
                 Nrrr_prime * r_prime**3)

    if data['nb_rud'] == 1:
        # Propeller hydrodynamic derivative calculation
        betaP = beta - data['xP'] / data['L'] * r_prime

        # Wake factor calculation
        if 'wP0' in data:
            if 'C1' in data and 'C2Minus' in data and 'C2Plus' in data:
                C2 = data['C2Plus'] if betaP > 0 else data['C2Minus']
                wP = 1 - (1 - data['wP0']) * (1 + (1 - np.exp(-data['C1'] * abs(betaP))) * (C2 - 1))
            else:
                wP = data['wP0'] * np.exp(-4 * betaP**2)
        elif 'wP' in data:
            wP = data['wP']
        else:
            raise ValueError("Missing wake factor data: 'wP0' or 'wP' must be provided.")

        JP = u * (1 - wP) / (data['nP'] * data['DP'])
        KT = data['k0'] + data['k1'] * JP + data['k2'] * JP**2
        TP = data['rho'] * data['nP']**2 * data['DP']**4 * KT
        XP = (1 - data['tP']) * TP

        # Rudder hydrodynamic derivatives calculation
        betaR = beta - data['lR_prime'] * r_prime
        gammaR = data['gammaPlus'] if betaR > 0 else data['gammaMinus']
        vR = U * gammaR * betaR
        eta = data['DP'] / data['HR']
        uR = (data['epsilon'] * u * (1 - wP) *
              np.sqrt(eta * (1 + data['kapa'] * (np.sqrt(1 + 8 * KT / np.pi / JP**2) - 1))**2 +
                      (1 - eta)))
        alphaR = delta - np.arctan(vR / uR)
        UR = np.sqrt(uR**2 + vR**2)
        falpha = data['lambda'] * 6.13 / (data['lambda'] + 2.25)
        Fn = 0.5 * data['rho'] * data['AR'] * UR**2 * falpha * np.sin(alphaR)

        if 'xR_prime' not in data:
            data['xR_prime'] = -0.5  # Default value

        XR = -(1 - data['tR']) * Fn * np.sin(delta)
        YR = -(1 + data['aH']) * Fn * np.cos(delta)
        xR = data['xR_prime'] * data['L']
        xH = data['xH_prime'] * data['L']
        NR = -(xR + data['aH'] * xH) * Fn * np.cos(delta)

    elif data['nb_rud'] == 2:
        raise NotImplementedError("The formula for 2 rudders is not implemented yet.")

    # Equation of motion system
    FX = (XH + XR + XP) + vm * M[1, 1] * r + r**2 * data['xG'] * m
    FY = (YH + YR) - M[0, 0] * u * r
    FN = (NH + NR) - data['xG'] * m * u * r
    F = np.array([FX, FY, FN])

    # Solve for Z_dot
    Z_dot = np.linalg.solve(M, F)

    return Z_dot