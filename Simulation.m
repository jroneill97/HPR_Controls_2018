clc; clear all; close all;
load rocket;
load motorCluster;
%% References
% -----------------------------------------------
% ---------- Reference for state array ---------- 
% -----------------------------------------------
% position         - 1:3  - x y z
% velocity         - 4:6  - u v w
% angular position - 7:10 - q1 q2 q3 q4
% angular velocity - 11:13

%% --------- Define initial states, time span, and resolution here --------
states = zeros(1,13); % initialize state matrix

initial_yaw   = 0.0;
initial_pitch = 0.0;
initial_roll  = 0.0;
states(7:10)  = angle2quat(initial_yaw,initial_pitch,initial_roll,'ZYX');
% quaternion in which the scalar part is the first index
states(11:13) = [0.0 0.0 0.0];
states(6)     = 0.0001;
states(5)     = 5;
states(4)     = 5;

t0       = 0;        % Initial Time
tf       = 50;       % Final Time
nip      = 2;        % Number of integration points
nsteps   = 1000;      % Number of steps between t0 and tf ("resolution")

t = t0;         % initialize t
% -------------------------------------------------------------------------
%% Define the initial state and time span arrays
stepSize = tf/nsteps;
tspan    = [t0:stepSize:tf]'; % Total time span
currentStates = states';            % State array used inside the loop

%% Re-map the thrust data to fit the time span array
motorCluster = CreateThrustCurves(motorCluster,tspan);

%% Solve the equations of motion
numMotors      = size(motorCluster); % size is the number of rocket motors
options        = odeset('JConstant','on', 'RelTol',1e-6, 'AbsTol',1e-6);

for i = 1:nsteps 
    t1 = stepSize*(i-1);
    t2 = stepSize*i;
    temp_tspan = t1:(t2-t1)/nip:t2;
    
%% Find the thrust at the current time
    [~,index] = min(abs(t2-tspan));
    for j = 1:numMotors(2)
      currentThrust(j) = motorCluster(j).enable * motorCluster(j).thrust(index); % Thrust in the inertial z axis. the Enable 
                                                                            % component sets the thrust for that motor
                                                                            % to zero if enable is set to 0.
    end   
%% Solve the ODE    
    [tNew,tempStates] = ode45(@(tNew,currentStates) EquationsOfMotion(currentStates,rocket,motorCluster,...
                                                                 currentThrust,...
                                                                 [0;0;0;0]),...
                                                                 temp_tspan,currentStates,options);
    
    t(i) = t2;
    currentStates = tempStates(nip+1,1:13)';
    states(i,:) = currentStates';
    
%% Break conditions
    % break if apogee is reached
    if states(i,6) <= 0
        %clc;
        disp('Apogee reached');
        break
    end
    
    % break if angle is > 50 degrees
    [yaw(i),pitch(i),roll(i)] = quat2angle(states(i,7:10),'ZYX');
    if ((abs(pitch(i)) > 0.8727) || (abs(roll(i)) > 0.8727)) && (norm(states(i,4:6)) > 20)
        %clc;
        disp('Abort: pitch or roll exceeded 50 degrees!');
        break
    end
    disp(t2);
end
clearvars -except t states stepSize rocket motorCluster yaw pitch roll

%% Animate the resulting state array
zoom = 200; % Distance from camera to the rocket (m)

AnimateRocket(t,states,rocket,zoom,'follow'); % 'follow' or 'stationary'AnimateRocket(t,states,rocket,zoom,'follow'); % 'follow' or 'stationary'
