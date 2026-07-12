# Hovering Cube Example
#### Imagine a cube with these properties:
* A flight computer inside.
* Two error-prone altimeters (each accurate to +/- ~2m, sample rate = 5Hz)
* 4 evenly space thrusters on the bottom, each with a max force.

<img width="321" height="322" alt="image" src="https://github.com/user-attachments/assets/c8496521-2e77-46e7-8d6b-7d79df5d5e7d" />

#### Conditions:
* At t=15s, one of the thrusters breaks and shuts down.
* At t=35s, altimeter 1 (the much better one) fails.

#### The goal?
* The cube starts at a height of 20m, then suddenly drops.
* The cube must drop, then hover at 5m even with the faults.
