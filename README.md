# MMG 3DOF Ship Maneuvering Simulation

A Matlab project simulating ship maneuvering motion in calm water using the **MMG 3DOF model** of Yasukawa &amp; Yoshimura [1]. The simulation solves the 3-degree-of-freedom equations of motion at midship (surge, sway, yaw) with a **4th-order Runge-Kutta solver**. Turning and zigzag motion simulations are available.

---

## Project Structure

- **`main`**: Initializes, runs the simulation, and plots results.
- **`solver`**: Implements the Runge-Kutta solver.
- **`MMG_3DOF`**: Defines the equations of motion (`f(X, t)`) 
- **Input Files (`.txt`)**: Contains ship parameters: dimensions, initial state, propeller/rudder characteristics and hull data.

## How to Run

1. Ensure input `.txt` files are in the project directory.
2. Run the main file
3. Midship results (trajectories, velocities) are plotted automatically 

---

## Input file


Input files (.txt) define ship-specific parameters.

main dimension: ship’s length, beam, mean draught, block coefficient, longitudinal position of 
center of gravity 

Hydrostatic properties: Dimensionless added mass and added moment of inertia

Initial values: initial/cruise ship speed, initial rudder angle, rate of turn of rudder, Cruise propeller rate of turn

Hydrodynamic coefficients: Dimensionless values derived from experiments or empirical formulas.

Propeller parameters: propeller diameter, thrust deduction factor, longitudinal positions of propeller, propeller open water coefficients and wake coefficient.
Wake coefficient in straight motion wP0 and wake constants C1 and C2 are optional for a more precise wake calculation  

Rudder parameters: number of rudder, rudder span, factor of lateral force due to steering, dimensionless application point of lateral force factor due to steering, deduction factor due to rudder resistance, ratio of wake fraction at propeller and rudder positions, experimental constant for expressing rudder longitudinal incoming flow,  flow straightening factor due to yaw motion, rudder aspect ratio,profile area of moveable part of rudder, flow straightening factor due to sway motion for both starboard and port side turnings, and the dimensionless longitudinal positions of rudder (set to -0.5 if not filled)
⚠ Only 1 rudder is currently supported


Data for KVLCC2 are from reference[1] 

---

## Validation

The code results  (Sim) for KVLCC2-L7-model are compared with experimental data (Exp-Ref) and simulation (Sim-Ref) results from [1]. 

### Turning (35° &amp; -35°)


| Parameters         | Exp-Ref (+35) | Sim-Ref (+35) | Sim (+35) | Exp-Ref (-35) | Sim-Ref (-35) | Sim (-35) |
| ------------------ | ------------- | ------------- | --------- | ------------- | ------------- | --------- |
| Advance' (-)       | 3.25          | 3.31          | 3.43      | 3.11          | 3.26          | 3.28      |
| Tactical Diam' (-) | 3.34          | 3.36          | 3.41      | 3.08          | 3.26          | 3.14      |

### Zigzag (10°/-10°)


| Parameters  | Exp-Ref (10/10) | Sim-Ref (10/10) | Sim (10/10) | Exp-Ref (-10/-10) | Sim-Ref (-10/-10) | Sim (-10/-10) |
| ----------- | --------------- | --------------- | ----------- | ----------------- | ----------------- | ------------- |
| OSA 1 (deg) | 8.20            | 5.20            | 5.86        | 9.50              | 7,60              | 8.22          |
| OSA 2 (deg) | 21.9            | 15.8            | 15.7        | 15.0              | 10.2              | 10.51         |

### Zigzag (20°/-20°)


| Parameters  | Exp-Ref (20/20) | Sim-Ref (20/20) | Sim (20/20) | Exp-Ref (-20/-20) | Sim-Ref (-20/-20) | Sim (-20/-20) |
| ----------- | --------------- | --------------- | ----------- | ----------------- | ----------------- | ------------- |
| OSA 1 (deg) | 13.7            | 10.9            | 12.3        | 15.1              | 14.5              | 16.0          |

---

## Reference

[1] Yasukawa, H., Yoshimura, Y. (2015). Introduction of MMG standard method for ship maneuvering predictions. J Mar Sci Technol 20, 37–52. https://doi.org/10.1007/s00773-014-0293-y
