clc; clear all; close all;
load rocket;
load motorCluster;
%% Define parameters used throughout the simulation
global rho g Fg_i
rho  = 1.225;
g    = 9.8;
Fg_i = [0; 0; -g*rocket.m]; % force of gravity in the Inertial Frame

% -----------------------------------------------
% ---------- Reference for state array ---------- 
% -----------------------------------------------
% position         - 1:3 - x y z
% velocity         - 4:6 - u v w
% angular position - 7:10 - q1 q2 q3 q4
% angular velocity - 11:13

%% --------- Define initial states, time span, and resolution here --------
states = zeros(1,13); % initialize state matrix

initial_yaw   = 0.0;
initial_pitch = 0.0;
roll  = 0.0;
states(7:10) = circshift(angle2quat(initial_yaw,initial_pitch,roll),-1); % Using ZYX for all rotations
states(11:13)= [0.0 0.0 1.0];
states(1) = 0;
states(6) = 0.0;

t0       = 0;    % Initial Time
tf       = 50;   % Final Time
nip      = 2;    % Number of integration points
nsteps   = 1000; % Number of steps between t0 and tf ("resolution")

t = t0;         % initialize t
% -------------------------------------------------------------------------
%% Define the initial state and time span arrays
stepSize = tf/nsteps;
tspan0    = [t0:stepSize:tf]'; % Total time span
statesIC = states';            % State array used inside the loop

%% Re-map the thrust data to fit the time span array
motorCluster = CreateThrustCurves(motorCluster,tspan0);

%% Solve the equations of motion
options = odeset('JConstant','on', 'RelTol',1e-6, 'AbsTol',1e-6);
for i = 1:nsteps  
    t1 = stepSize*(i-1);
    t2 = stepSize*i;
    temp_tspan = t1:(t2-t1)/nip:t2;
    % can include if statement to stop calling this function
    temp_thrustCurve = CreateThrustCurves(motorCluster,temp_tspan);
    
    [tNew,tempStates] = ode45(@(tNew,statesIC) EquationsOfMotion(tNew,...
                              statesIC,temp_thrustCurve,temp_tspan,rocket),...
                              temp_tspan,statesIC,options);
    
    t(i) = t2;
    statesIC = tempStates(nip+1,1:13)';
    states(i,:) = statesIC';
    
    disp((i*100)/nsteps); % display percent completion
end
clearvars -except t states stepSize rocket
%% Display some data
clc;
maxSpeed     = norm([max(states(:,4)) max(states(:,5)) max(states(:,6))]);
apogee       = max(states(:,3));
disp('Maximum speed');
disp(maxSpeed);
disp('Altitude at apogee');
disp(apogee);

%% Animate the resulting state array
whitebg([1 1 1]);
close all;
zoom = 15; % Distance from camera to the rocket (m)
AnimateRocket(t,states,rocket,zoom,'stationary'); % 'follow' or 'stationary'
     