clc; clear all; close all;

% Define parameters used throughout the simulation
global rho g
rho = 1.225;
g   = -9.8;
load('rocket.mat');

% ------------------------------------------------------------------------
% position     - 1:3 - x y z
% velocity     - 4:6 - u v w
% ang position - 7:10 - q1 q2 q3 q4
% ang velocity - 11:14

t = 0;               % initialize t 
states = zeros(1,14); % initialize state matrix

% User-defined initial states
states(7)  = 50;

% User-defined time span and resolution
t0 = 0;
tf = 10;

nsteps   = 50; % Number of steps between t0 and tf
stepSize = 1/nsteps;
tspan    = t0:stepSize:tf; % Time span created from defined step size

nip      = 100; % Number of integration points
statesIC = states;   % State array used inside the loop

options = odeset('JConstant','on', 'RelTol',1e-6, 'AbsTol',1e-6);
for i = 1:nsteps  
    t1 = tf*(i-1)/nsteps;
    t2 = tf*i/nsteps;
    tspan = t1:(t2-t1)/nip:t2;
    
    [tNew,tempStates] = ode45(@(tNew,xIC) EquationsOfMotion(tNew,xIC,thrust,rocket),...
                               tspan,statesIC,options);
                           
    t(i) = t2;
    statesIC = tempStates(nip+1,1:14)';
    statesNew(i,:) = statesIC';
    
    disp((i*100)/nsteps); % display percent completion
end
states = statesNew;
% Animate the resulting state array
ax = axes('XLim',[0 10],'YLim',[-11 11],'ZLim',[0 122]);

AnimateRocket(t,states,ax,rocket);