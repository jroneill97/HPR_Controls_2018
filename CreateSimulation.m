clc; clear all; close all;

%% Define parameters used throughout the simulation
global rho g
rho = 1.225;
g   = -9.8;
load('rocket.mat');
load('Thrust_Data.mat');
% -----------------------------------------------
% ---------- Reference for state array ---------- 
% -----------------------------------------------
% position         - 1:3 - x y z
% velocity         - 4:6 - u v w
% angular position - 7:10 - q1 q2 q3 q4
% angular velocity - 11:13

%% --------- Define initial states, time span, and resolution here --------
states = zeros(1,13); % initialize state matrix

yaw   = 0.0;
pitch = 0.05;
roll  = 0.0;
states(7:10) = circshift(angle2quat(yaw,pitch,roll),-1); % Using ZYX for all rotations
states(11:13)= [0.0 0.0 0.0];
states(1) = 0;
states(6) = 0.0;

t0 = 0;         % Initial Time
tf = 10;        % Final Time
nip      = 50;   % Number of integration points
nsteps   = 250; % Number of steps between t0 and tf ("resolution")

t = t0;          % initialize t
% -------------------------------------------------------------------------
%% Define the initial state and time span arrays
stepSize = 1/nsteps;
tspan0    = [t0:stepSize:tf]'; % Total time span
statesIC = states';          % State array used inside the loop

%% Re-map the thrust data to fit the time span array
thrust_H130 = interp1(H130.time,H130.thrust,tspan0);
thrust_I170 = interp1(I170.time,I170.thrust,tspan0);
for i = 1:length(tspan0)
    if isnan(thrust_H130(i))
        thrust_H130(i) = 0;
    end
    if isnan(thrust_I170(i))
        thrust_I170(i) = 0;
    end
end

%% Solve the equations of motion
options = odeset('JConstant','on', 'RelTol',1e-6, 'AbsTol',1e-6);
for i = 1:nsteps  
    t1 = tf*(i-1)/nsteps;
    t2 = tf*i/nsteps;
    tspan = t1:(t2-t1)/nip:t2;
    thrust = interp1(tspan0,thrust_H130,tspan,'spline');
    
    [tNew,tempStates] = ode45(@(tNew,statesIC) EquationsOfMotion(tNew,statesIC,thrust,tspan,rocket),...
                               tspan,statesIC,options);
    
    t(i) = t2;
    statesIC = tempStates(nip+1,1:13)';
    states(i,:) = statesIC';
    
    disp((i*100)/nsteps); % display percent completion
end

%% Animate the resulting state array
    whitebg([0 .5 .6])
    AnimateRocket(t,states,stepSize,rocket);