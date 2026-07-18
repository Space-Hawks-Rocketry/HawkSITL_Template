# Hovering Cube Example

#### Key Lessons
This example demonstrates the following techniques:
* Modeling 1D vehicle dynamics in Python.
* Modeling sensor noise in Python (though not accurately).
* Encorporating models into the SITL environment.
* Sending sensor readings from the environment to the flight computer.
* Sending control commands from the flight computer to the environment.
* Driving environment dynamics with control commands received from the flight computer (closed-loop SITL).
* Combining sensor measurements in a Kalman filter to estimate altitude.
* Using a PD controller to control thrusters for targeted hovering.
* Implementing a very basic state machine into a flight computer.
* Measuring sensor variance during calibration.
* Saving and plotting simulation data.

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
* Use a state machine that switches between "CALIBRATING" and "READY"
* Determine altimeter variances during calibration stage.
  * Their relative accuracies will be used for Kalman filtering and weighted averaging.
* Use a Kalman filter for altitude estimation that combines altimeters with known-ish thrust magnitude.
  * Measurements must be sequentially updated to account for different altimeter sample rates.
* Use a PD controller to adjust thruster throttles based on distance from altitude goal.

#### Result
<img width="439" height="272" alt="image" src="https://github.com/user-attachments/assets/255b7a39-4a9a-44a0-af7c-910be229bd33" />

Before t=40 (altimeter 1 failure), the flight computer's estimated altitude is quite close to "reality" (sim-truth). The removal of altimeter 1 at t=40 draastically reduces the accuracy of the altitude estimate, and drives the thrust controller to correct for false altitude differences. Adding other sensors such as an IMU could help dramatically in these situations.

Overall, the PD controller could use some tuning to prevent the unusual overcorrections in the thrusters, but the Kalman filter does a very good job of combining altimeter readings and known control commands to get an accurate picture of the cube's altitude.
