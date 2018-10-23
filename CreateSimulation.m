clc; clear all; close all;

%% Define parameters used throughout the simulation
global rho g
rho = 1.225;
g   = -9.8;
load rocket;
load motorCluster;
% -----------------------------------------------
% ---------- Reference for state array ---------- 
% -----------------------------------------------
% position         - 1:3 - x y z
% velocity         - 4:6 - u v w
% angular position - 7:10 - q1 q2 q3 q4
% angular velocity - 11:13

%% --------- Define initial states, time span, and resolution here --------
states = zeros(1,13); % initialize state matrix

yaw   = 0.00001;
pitch = 0.0;
roll  = 0.0;
states(7:10) = circshift(angle2quat(yaw,pitch,roll),-1); % Using ZYX for all rotations
states(11:13)= [0.01 -0.01 0.0];
states(1) = 0;
states(6) = 0.0;

t0       = 0;   % Initial Time
tf       = 10;  % Final Time
nip      = 20;  % Number of integration points
nsteps   = 1000; % Number of steps between t0 and tf ("resolution")

t = t0;         % initialize t
% -------------------------------------------------------------------------
%% Define the initial state and time span arrays
stepSize = 1/nsteps;
tspan0    = [t0:stepSize:tf]'; % Total time span
statesIC = states';          % State array used inside the loop

%% Re-map the thrust data to fit the time span array
motorCluster = CreateThrustCurves(motorCluster,tspan0);

%% Solve the equations of motion
options = odeset('JConstant','on', 'RelTol',1e-6, 'AbsTol',1e-6);
for i = 1:nsteps  
    t1 = tf*(i-1)/nsteps;
    t2 = tf*i/nsteps;
    temp_tspan = t1:(t2-t1)/nip:t2;
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

%% Animate the resulting state array
    whitebg([0 0 0]);
    zoom = 50; % Distance from camera to the rocket (m)
    AnimateRocket(t,states,rocket,zoom,'follow'); % 'follow' or 'stationary'
     