# Hovering Cube Example
#### Imagine a cube with these properties:
* A flight computer inside.
* Two error-prone altimeters:
  * Altimeter 1 @ 30Hz, with an undetermined variance.
  * Altimeter 2 @ 5Hz, with an undetermined variance much larger than altimeter 1's.
* 4 evenly space thrusters on the bottom, each with a max force of 15N.
  * The valves are cheap, and their actual state ("openess") has a 0.0025 variance from the control state.

<img width="321" height="322" alt="image" src="https://github.com/user-attachments/assets/c8496521-2e77-46e7-8d6b-7d79df5d5e7d" />

#### Conditions:
* At t=0s, the flight computer enters a calibration state.
* At t=5s, the flight computer finishes calibration, and the cube is released.
* At t=20s, one of the thrusters breaks and shuts down.
* At t=40s, altimeter 1 (the nicer one) fails.

#### The goal?
* The cube is suspended 20m off the ground, then gets dropped after calibrating.
* The cube must drop, then hover at 5m even with sensor and altimeter faults.
  * This requires good altitude estimation.

#### Strategy
* Use a state machine that switches between "Calibrating", "Ready", and "Armed"
* Determine altimeter variances during calibration stage.
  * Their relative accuracies will be used for Kalman filtering and weighted averaging.
* Use a Kalman filter for altitude estimation that combines altimeters with known-ish thrust magnitude.
  * Measurements must be sequentially updated to account for different altimeter sample rates.
