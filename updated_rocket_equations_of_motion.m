function states_i_dot = rocket_equations_of_motion(t,y)

%% define global variables
global rho        % air density
global g          % force of gravity (earth fixed)

global T          % set thrust to 0
global area       % area matrix  [nose side side]
global body_diam  % body diameter
global dcgref     % distance from something to something 
global dcg        % distance from bottom of rocket to cg

global m          % total mass
global I          % I matrix     diag([Ix Iy Iz])
global L

global Cl         % Rolling moment coefficient
global Cm         % Pitching moment coefficient
global Cn         % Yawing moment coefficient

global CL         % Coefficient of moment due to roll rate
global CM         % Coefficient of moment due to pitch rate
global CN         % Coefficient of moment due to yaw rate

global CA         
global CY  
global Lap
global Lay
global Lda
global delta

%% inertial states
states_i = y;

pos_i  = states_i(1:3);
vel_i  = states_i(4:6);
ang_i  = states_i(7:9); % (roll pitch yaw)
angV_i = states_i(10:12);

%% Calculate Angle of Attack
yaw   = ang_i(3);
pitch = ang_i(2);
roll  = ang_i(1); % rotating around length of rocket

alpha = atan2(vel_i(3),vel_i(1));
beta  = asin(vel_i(1)/norm(vel_i));

ap    = atan2(vel_i(3),vel_i(1));
ay    = atan2(vel_i(2),vel_i(1));

%% Body-fixed angular velocity (P,Q,R)
rotationMat = [1  0         -sin(pitch);
               0  cos(roll)  cos(pitch)*sin(roll);
               0 -sin(roll)  cos(pitch)*cos(roll)];
angVel_b = rotationMat*angV_i;

P = angVel_b(1);
Q = angVel_b(2);
R = angVel_b(3);
%% Aerodynamic coefficients
q  =  0.5*rho*norm(vel_i)^2;  % dynamic pressure

Clap = Lap/(q*area*body_diam); % slope of rolling moment due to pitch
Clay = Lay/(q*area*body_diam); 
Clda = Lda/(q*area*body_diam);
                 
%% Moments due to Aerodynamics
L_b = Cl*q*area*body_diam;  %q*area*body_diam*(Clap*ap+Clay*ay+Clda*delta);
M_b = Cm*q*area*L;          %q*area*body_diam*((Cm+CM*(Q*body_diam)./(2*norm(vel_i)))+CM*((dcg-dcgref)/body_diam));
N_b = Cn*q*area*body_diam;  %q*area*body_diam*((Cn+CN*(R*body_diam)./(2*norm(vel_i)))+CY*((dcg-dcgref)/body_diam));

Mom_b = [L_b;M_b;N_b];

%% Total translational force on rocket
Fx_b = T - m*g*sin(pitch)           - CA*q*area; % Total force in x direction (body)
Fy_b =     m*g*sin(roll)*cos(pitch) - CY*q*area; % Total force in y direction (body)
Fz_b =     m*g*cos(roll)*cos(pitch) - CN*q*area; % Total force in z direction (body)

F_b  = [Fx_b;Fy_b;Fz_b]; % body-fixed translational force on rocket

%% Rotation matrices to convert body-fixed to inertial frames
A=[cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);...
   cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)+cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)*sin(roll)*sin(yaw);...
   -sin(pitch) sin(roll)*cos(pitch) cos(roll)*cos(pitch)];

B=[1 sin(roll)*tan(pitch) cos(roll)*tan(pitch);...
   0 cos(pitch)           -sin(roll)          ;...
   0 sin(roll)/cos(pitch) cos(roll)./cos(pitch)];

C=[0 -R  Q;...
   R  0 -P;...
  -Q  P  0];

%% Create time derivative of state matrix
pos_i_dot=A*vel_i;                 % inertial position
ang_i_dot=B*angVel_b;                % inertial angular velocity
vel_i_dot=(F_b/m)-C*vel_i;       % inertial acceleration (transport theorem)
angV_i_dot=I\(Mom_b-C*I*angVel_b); % inertial angular acceleration

states_i_dot = [pos_i_dot; vel_i_dot; ang_i_dot; angV_i_dot];