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

The code was verified using experimental data from [2]. The comparison between experimental and simulated results for maneuvering tests is summarized below. They show errors lower than 20% in maximum. 

### Turning (35° &amp; -35°)


| Parameter         | exp (stb) | sim (stb) | error (%) | exp (port) | sim (port) | error (%) |
| ----------------- | --------- | --------- | --------- | ---------- | ---------- | --------- |
| Advance (-)       | 3.28      | 3.34      | 2%        | 3.19       | 3.20       | 0%        |
| Turning Diam. (-) | 3.28      | 3.23      | 2%        | 3.07       | 2.97       | 3%        |
| Transfer (-)      | 1.3       | 1.43      | 10%       | 1.17       | 1.31       | 12%       |


### Zigzag (10°/-10°)


| Parameter   | exp (stb) | sim (stb) | error (%) | exp (port) | sim (port) | error (%) |
| ----------- | --------- | --------- | --------- | ---------- | ---------- | --------- |
| OSA 1 (deg) | 8.4       | 5.63      | 33%       | 10         | 10.27      | 3%        |
| OSA 2 (deg) | 19.6      | 19.21     | 2%        | 16.1       | 15.83      | 2%        |


### Zigzag (20°/-20°)


| Parameter   | exp (stb) | sim (stb) | error (%) | exp (port) | sim (port) | error (%) |
| ----------- | --------- | --------- | --------- | ---------- | ---------- | --------- |
| OSA 1 (deg) | 13.9      | 15.578    | 12%       | 15.4       | 15.35      | 0%        |
| OSA 2 (deg) | 15.5      | 18.353    | 18%       | 13.2       | 15.07      | 14%       |

---

## Reference

[1] Yasukawa, H., Yoshimura, Y. Introduction of MMG standard method for ship maneuvering predictions. J Mar Sci Technol 20, 37–52 (2015). https://doi.org/10.1007/s00773-014-0293-y

[2] Yoshimura, Y., Ueno, M., & Tsukada, Y. (2008). Analysis of steady hydrodynamic force components and prediction of manoeuvring ship motion with KVLCC1, KVLCC2 and KCS. In Workshop Proceedings of SIMMAN2008 (Vol. 1, pp. E80-E86).