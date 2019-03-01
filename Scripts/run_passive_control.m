% Main launch simulator
% Authors: WPI Flight Dynamics and Stability MQP team
% Date:    02/28/2019
clc; clear; close all;
addpath('../Functions');
addpath('../Data_Files');
options = odeset('JConstant','on', 'RelTol',1e-4, 'AbsTol',1e-4);
load rocket;
load wind;
load parachutes;
%% Define your initial conditions and user inputs (inertial reference frame)
initial_pos      = [0.0 0.0 0.0];  % Initial translational position
initial_euler    = [0.0 0.0 0.0];  % Initial euler angles
initial_vel      = [0.0 0.0 0.0];  % Initial velocity
initial_omega    = [0.0 0.0 0.0];  % Initial rotational velocity
load motorCluster_v3;              % Choose motor configuration. (v1, v2, v3)
motor_enable     = [1];            % Enable array (length must = # of motors)
simulate_landing = 1;              % 1: simulate descent 0: break at apogee
include_wind     = 1;              % 1: include 0: do not include
animation_type   = 'plot';         % 'plot','plot_circle','follow','stationary'
%% Initialize
states = zeros(1,13); % initialize state matrix
states(1:3)   = initial_pos';
states(4:6)   = initial_vel'; states(6) = states(6)+0.0001;
states(7:10)  = angle2quat(initial_euler(3)+0.001,initial_euler(2),initial_euler(1),'ZYX');
states(11:13) = [0.0 0.0 0.0];

parachuteDeployed = false;
t0       = 0;        % Initial Time
tf       = 300;      % Final Time (not necessary to adjust)
nip      = 2;        % Number of integration points
nsteps   = 10000;
t        = t0;       % initialize t
stepSize = tf/nsteps;
tspan    = [t0:stepSize:tf]';       % Total time span
currentStates = states';            % State array used inside the loop
motorCluster = create_thrust_curves(motorCluster,tspan);
numMotors      = size(motorCluster); % size is the number of rocket motors
for i = 1:numMotors(2)
motorCluster(i).enable = motor_enable(i);
end
%% Begin solving for flight trajectory
for i = 1:nsteps 
    t1 = stepSize*(i-1);
    t2 = stepSize*i;
    temp_tspan = t1:(t2-t1)/nip:t2;   
%% Find the thrust at current time
    [~,index] = min(abs(t2-tspan));
    for j = 1:numMotors(2)
      currentThrust(j) = motorCluster(j).enable * motorCluster(j).thrust(index); % Thrust in the inertial z axis. the Enable 
                                                                                 % component sets the thrust for that motor
                                                                                 % to zero if enable is set to 0.
    end       
%% Find the wind velocity at current altitude
    [~,index] = min(abs(currentStates(3)-wind.altitude));
      currentWindVel = include_wind*wind.velocity(index);
%% Solve the ODE    
switch parachuteDeployed
    case false
    [tNew,tempStates] = ode45(@(tNew,currentStates) equations_of_motion(currentStates,rocket,motorCluster,...
                                                                 currentThrust,currentWindVel,...
                                                                 [0;0;0;0]),...
                                                                 temp_tspan,currentStates,options);
    case true
    break
    [tNew,tempStates] = ode45(@(tNew,currentStates) main_chute_equations_of_motion(currentStates,rocket,mainChute),...
                                                                 temp_tspan,currentStates,options);
end
    t(i) = t2;
    currentStates = tempStates(nip+1,1:13)';
    states(i,:) = currentStates';   
%% Break conditions
    % break if apogee is reached
    if (states(i,6) <= 0) && (parachuteDeployed == false)
        fprintf("Apogee reached at %0.1f meters \n",states(i,3));
        pause(1);
        parachuteDeployed = true;
        if simulate_landing == 0
            break;
        end
    end
    if (parachuteDeployed == true) && (states(i,3) <= 0)
        fprintf("Rocket has landed %0.0f meters away from launch site at %0.1f m/s\n",norm(states(i,1:2)),states(i,6));
        break;
    end
fprintf("time = %0.1f\n",t2);
end
%% Animate the resulting state array
animate_rocket(t,states,rocket,50,animation_type);
fprintf("Animation Complete \n");

