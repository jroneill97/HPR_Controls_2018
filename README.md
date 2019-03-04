# High Powered Rocketry - Major Qualifying Project
## Flight Dynamics and Stability Team
## Worcester Polytechnic Insitute - 2018-2019
# Introduction:
(Check back later for link to full report)

The baseline primary goal of the overall project was to design a Class I model rocket to fly to an altitude of 1500 feet and return safely to the ground. Through the collaboration of the entire MQP team, we outlined secondary goals to further challenge ourselves for this project. The secondary goals included the use of CO2 stage separation, electromagnetic booster separation, an autorotation recovery system and actuated fins.

To complete these tasks, the MQP team was broken in to three sub-teams: The Mechanical, Structural Analysis and Thermal (MSAT) team, the Propulsion, Staging, and Recovery (PSR) team, and our team, the Flight Dynamics and Stability (Controls) team. The MSAT team was in charge of designing the rocket as well as providing any structural and thermal analysis of the rocket body. The PSR team lead the design and development of the CO2 separation system, the electromagnetic booster separation, and the autorotation recovery system. The Controls team was in charge of the design and development of actuated fins as well as developing a simulation of the rocket’s flight and electrically integrating avionics and electronic systems onboard the rocket.

The Controls team had many tasks at hand, the most challenging of which being the design and development of actuated fins. This involved optimizing fin design and developing a control law so the fins reacted properly to disturbances in flight. Other tasks included using MATLAB to build a flight simulation and motor failure plots and electronically integrating avionics and electronic instruments to the rocket to ensure proper timing of stage and booster separation.
# MATLAB Trajectory Simulator
## Creating a Launch Simulation:
  - Open the following file: /Scripts/run_passive_control.m
  - To change the rocket physical parameters, go to: /Data_Files/rocket.mat and save any changes
  - The simulator will run with default initial conditions. If you wish to change any of the initial conditions (inertial reference frame), update the following lines of code:
```
initial_pos      = [0.0 0.0 0.0];  % Initial translational positioe
initial_euler    = [0.0 0.0 0.0];  % Initial euler angles (roll, pitch, yaw)
initial_vel      = [0.0 0.0 0.0];  % Initial velocity
initial_omega    = [0.0 0.0 0.0];  % Initial rotational velocity
load motorCluster_v3;              % Choose motor configuration (v1, v2, v3)
motor_enable     = [1];            % Array elements of 1 or 0 (enable, disable)
simulate_landing = 1;              % 1: simulate descent 0: break at apogee
include_wind     = 1;              % 1: include 0: do not include
plot_type        = 'plot';         % 'plot','plot_circle','follow','stationary'
```

## Notes on Trajectory Simulator:
  - You will need to install the Mathworks Aerospace Toolbox
  - There are three motor configuration structs with suffixes (```v1```, ```v2```, and ```v3```)
    - ```v1```: Original 7-motor cluster of three H130 central motors with four I170 boosters
    - ```v2```: Three H130 central motors
    - ```v3```: One I218 motor
    - The array ```motor_enable``` must be a 1xn array where n = number of rocket motors in the current motor configuration. Allows you to disable specific rocket motors in the cluster to simulate a motor failure to ignite.
  - Set ```simulate_landing``` to ```0``` if you do not want to simulate the rocket's parachute recovery stage
  - Set ```include_wind``` to ```0``` if you wish not to include the wind model in the simulation
  - Set ```plot_type``` to your preference of animation type:
    - ```'plot'```       : 3D plot of rocket trajectory
    - ```'plot_circle'```: Plots a circle with radius equal to the landing distance
    - ```'follow'```     : Animates the rocket along its flight path (useful for visualizing its rotational motion)
  
  # Flight Controller - Details
  ## Hardware
  - Processor 
    - ATmega2560 (Mega CORE)
  - Sensors
    - BNO055 9DOF I2C IMU
    - MPL3115A2 I2C barometric pressure sensor
    - 433 MHz RF transmitter
  - Power Supply
    - Turnigy 1000mAh 3S 20C LiPo
  ## Libraries
  - https://github.com/adafruit/Adafruit_MPL3115A2_Library
  - https://github.com/adafruit/Adafruit_BNO055
