# High Powered Rocketry Major Qualifying Project
## Flight Dynamics and Stability Team
## 2018-2019

# Creating a Launch Simulation:
- Open the following file: /Scripts/run_passive_control.m
- To change the rocket physical parameters, go to: /Data_Files/rocket.mat and save any changes
- The simulator will run with default initial conditions. If you wish to change any of the initial conditions, update the following lines of code:
```
initial_pos      = [0.0 0.0 0.0];  % Initial translational position
initial_euler    = [0.0 0.0 0.0];  % Initial euler angles (roll, pitch, yaw)
initial_vel      = [0.0 0.0 0.0];  % Initial velocity
initial_omega    = [0.0 0.0 0.0];  % Initial rotational velocity
load motorCluster_v3;              % Choose motor configuration (v1, v2, v3)
motor_enable     = [1];            % Enable array (length must = # of motors)
simulate_landing = 1;              % 1: simulate descent 0: break at apogee
include_wind     = 1;              % 1: include 0: do not include
plot_type        = 'plot';         % 'plot','plot_circle','follow','stationary'
```

# Notes:
- There are three motor configuration structs with suffixes (```v1```, ```v2```, and ```v3```)
  - ```v1```: Original 7-motor cluster of three H130 central motors with four I170 boosters
  - ```v2```: Three H130 central motors
  - ```v3```: One I218 motor
- The array ```motor_enable``` must be a 1xn array where n = number of rocket motors in the current motor configuration
- Set ```simulate_landing``` to ```0``` if you do not want to simulate the rocket's parachute recovery stage
- Set ```include_wind``` to ```0``` if you wish not to include the wind model in the simulation
- Set ```plot_type``` to your preference of animation type:
  - ```'plot'```       : 3D plot of rocket trajectory
  - ```'plot_circle'```: Plots a circle with radius equal to the landing distance
  - ```'follow'```     : Animates the rocket along its flight path (useful for visualizing its rotational motion)

# List of Directories
- Data_Files: Various .mat files used for simulator
- 