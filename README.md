# HawkSITL

## Table of Contents
1. [What is SITL?](#what-is-sitl)
2. [Intro to HawkSITL Template](#intro-to-hawksitl-template)

## What is SITL?
SITL (or software-in-the-loop) is a type of simulation capable of testing flight software before ever launching. 
Instead of on an embedded device, flight software is run on your computer, real hardware sensor data is replaced with simulation-generated
sensor readings, and the flight software sends control commands to the environment instead of to external hardware.

### SITL Enables:
* Quick software development cycle (no uploading code or sharing hardware).
* Sim-truth to compare against navigation computer estimates.
* More robust testing before flight.
  * Randomized environment conditions (such as wind).
  * Simulated sensor faults.
  * Completely free, break anything.
* Often closed-loop:
  * The environment affects the rocket.
  * Rocket control decisions affect the environment.

### SITL Limitations:
* Accuracy depends on environment models:
  * Atmospheric models
  * 6DOF model
  * Sensor models
  * Rocket model
* Cannot test hardware-interfacing firmware:
  * Sensor communication firmware.
  * Data logging firmware.

## Intro to HawkSITL Template
HawkSITL is the Space Hawks SITL framework enabling rapid flight software and GN&C prototyping. This repository serves as a template for
repositories attempting to use SITL testing for various projects.

### Goals of HawkSITL
In addition to typical SITL goals, we intend to:
* Make SITL projects portable between development environments, regardless of OS.
* Keep our framework lightweight and easy to use.
* Include basic models for general rocketry use.
* Enable easy transferability between SITL flight software and actual flight software.

### Architecture
The HawkSITL framework is split into two main parts:
* Environment (written entirely in **Python**)
* Flight Computer (written entirely in **C++**)

**SITL Communication Scheme**

<img width="449" height="244" alt="image" src="https://github.com/user-attachments/assets/ad151c4f-7d12-48e0-b907-15bfc0d82dff" />

*Made with Lucidchart.*

All communication exists user-defined JSON objects. HawkSITL requires no standards to be upheld, and only facilitates JSON communication so that
the framework user may supply sensor data and control commands in whatever format they choose.

#### User Directories
Most of the HawkSITL template is necessary framework code and should not be modified.

Here are the directories that you should work within (ideally):
* environment/simulation  -->  Define the environment models and simulation.
* flight-computer/src/simulation  --> Write flight software.
* flight-computer/include/simulation  --> Write flight software headers.

## Setup and Installation

<img width="498" height="376" alt="image" src="https://github.com/user-attachments/assets/14e991cb-b095-4422-bbc4-01151b514093" />
