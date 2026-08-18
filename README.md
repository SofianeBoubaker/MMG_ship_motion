# MMG 3DOF Ship Maneuvering Simulation

A Python project simulating ship maneuvering motion in calm water using the **MMG 3DOF model** of Yasukawa &amp; Yoshimura [1]. The simulation solves the 3-degree-of-freedom equations of motion (surge, sway, yaw) with a **4th-order Runge-Kutta solver**. Turning and zigzag motion simulations are available.

---

## Project Structure

- **`main`**: Initializes, runs the simulation, and plots results.
- **`solver`**: Implements the Runge-Kutta solver.
- **`MMG_3DOF`**: Defines the equations of motion (`f(X, t)`) 
- **Input Files (`.txt`)**: Contains ship parameters: dimensions, initial state, propeller/rudder characteristics and hull data.

## How to Run

1. Ensure input `.txt` files are in the project directory.
2. Run the main file
3. Results (trajectories, velocities) are plotted automatically.

---

## Input file


Input files (.txt) define ship-specific parameters.

main dimension: ship’s length, beam, mean draught, block coefficient, longitudinal position of 
center of gravity 

Hydrostatic properties: Dimensionless added mass and added moment of inertia

Initial values: initial/cruise ship speed, initial rudder angle, rate of turn of rudder, Cruise propeller rate of turn
⚠️ the rudder turning evolution is linear in the code

Hydrodynamic coefficients: Dimensionless values derived from experiments or empirical formulas.

Propeller parameters: propeller diameter, thrust deduction factor, longitudinal positions of propeller, effective wake fraction in straight motion and propeller open water coefficients

Rudder parameters: number of rudder, rudder span, factor of lateral force due to steering, application point of lateral force factor due to steering, deduction factor due to rudder resistance, ratio of wake fraction at propeller and rudder positions, experimental constant for expressing rudder longitudinal incoming flow, longitudinal positions of rudder, flow straightening factor due to yaw motion, rudder aspect ratio,profile area of moveable part of rudder, flow straightening factor due to sway motion for both starboard and port side turnings 
⚠️ only 1 rudder is currently supported

Data for KVLCC2 are from reference[1] and KVLCC1 are based on reference [2] 

---

## Validation

The code was verified using experimental data from [2]. The comparison between experimental and simulated results for KVLCC1 maneuvering tests is summarized below.

### Turning (35° &amp; -35°)


| Parameter          | exp (stb) | sim (stb) | error (%) | exp (port) | sim (port) | error (%) |
| ------------------ | --------- | --------- | --------- | ---------- | ---------- | --------- |
| Advance (-)        | 3.28      | 3.38      | 3.1%      | 3.19       | 3.25       | 1.8%      |
| Tactical Diam. (-) | 3.28      | 3.09      | 5.9%      | 3.07       | 2.87       | 6.4%      |
| Transfer (-)       | 1.3       | 1.38      | 6%        | 1.17       | 1.25       | 7%        |


### Zigzag (10°/-10°)


| Parameter   | exp (stb) | sim (stb) | error (%) | exp (port) | sim (port) | error (%) |
| ----------- | --------- | --------- | --------- | ---------- | ---------- | --------- |
| OSA 1 (deg) | 8.4       | 9.4       | 12%       | 10         | 9.7        | 3.2%      |
| OSA 2 (deg) | 19.6      | 24.2      | 23.6%     | 16.1       | 14.1       | 12.4%     |


### Zigzag (20°/-20°)


| Parameter   | exp (stb) | sim (stb) | error (%) | exp (port) | sim (port) | error (%) |
| ----------- | --------- | --------- | --------- | ---------- | ---------- | --------- |
| OSA 1 (deg) | 13.9      | 15.0      | 8%        | 15.4       | 21.6       | 40.5%     |
| OSA 2 (deg) | 15.5      | 19.5      | 26%       | 13.2       | 15.9       | 20%       |

---

## Reference

[1] Yasukawa, H., Yoshimura, Y. (2015). Introduction of MMG standard method for ship maneuvering predictions. J Mar Sci Technol 20, 37–52. https://doi.org/10.1007/s00773-014-0293-y

[2] Aksu, E., & Köse, E. (2017). Evaluation of Mathematical Models for Tankers' Maneuvering Motions. Journal of ETA Maritime Science, 5(1), 95-109. https://doi.org/10.5505/jems.2017.52523
