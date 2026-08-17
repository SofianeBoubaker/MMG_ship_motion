# MMG 3DOF Ship Maneuvering Simulation

A Python project simulating ship maneuvering motion in calm water using the **MMG 3DOF model** of Yasukawa &amp; Yoshimura [1]. The simulation solves the 3-degree-of-freedom equations of motion (surge, sway, yaw) with a **4th-order Runge-Kutta solver**. Turning and zigzag motion simulations are available.

---

## Project Structure

- **`main.py`**: Initializes, runs the simulation, and plots results.
- **`solver.py`**: Implements the Runge-Kutta solver.
- **`MMG_3DOF.py`**: Defines the equations of motion (`f(X, t)`) 
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

Hydrostatic properties: Added mass and added moment of inertia

Initial values: initial/cruise ship speed, initial rudder angle, rate of turn of rudder, initial/Cruise propeller rate of turn

Hydrodynamic coefficients: Derived from experiments or empirical formulas.

Propeller parameters: propeller diameter, thrust deduction factor, longitudinal positions of propeller, effective wake fraction in straight motion and propeller open water coefficients

Rudder parameters: number of rudder, rudder span, factor of lateral force due to steering, application point of lateral force factor due to steering, deduction factor due to rudder resistance, ratio of wake fraction at propeller and rudder positions, experimental constant for expressing rudder longitudinal incoming flow, longitudinal positions of rudder, flow straightening factor due to yaw motion, rudder aspect ratio, flow straightening factor due to sway motion 
for both starboard and port side turnings 

Example input files for KVLCC1 are based on reference [2] and [3]
---

Reference

[1] Yoshimura, Y., Yasukawa, H., Sakuno, R., (2019, February). Practical maneuvering simulation method of ships considering the roll-coupling effect 
DOI: https://doi.org/10.1007/s00773-014-0293-y

[2] Aksu, E., & Köse, E. (2017). Evaluation of Mathematical Models for Tankers' Manoeuvring Motions.

[3] Yoshimura, Y., Ueno, M., & Tsukada, Y. (2008). Analysis of steady hydrodynamic force components and prediction of manoeuvring ship motion with KVLCC1, KVLCC2 and KCS. In Workshop Proceedings of SIMMAN2008 (Vol. 1, pp. E80-E86).