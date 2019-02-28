# High Powered Rocketry Major Qualifying Project
## Flight Dynamics and Stability Team
## 2018-2019

# Creating a Launch Simulation
- Open the following file: /Scripts/run_passive_control.m
- To change the rocket physical parameters, go to: /Data_Files/rocket.mat and save any changes
- The simulator will run with default initial conditions. If you wish to change any of the initial conditions, update the following lines of code:
```
initial_pos      = [0.0 0.0 0.0];  % Initial translational position
initial_euler    = [0.0 0.0 0.0];  % Initial euler angles
initial_vel      = [0.0 0.0 0.0];  % Initial velocity
initial_omega    = [0.0 0.0 0.0];  % Initial rotational velocity
load motorCluster_v3;              % Choose motor configuration.
motor_enable     = [1];            % Enable array (length must = # of motors)
simulate_landing = 1;              % 1: simulate descent 0: break at apogee
include_wind     = 1;              % 1: include 0: do not include
animation_type   = 'plot';         % 'plot','plot_circle','follow','stationary'
```

