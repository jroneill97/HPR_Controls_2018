function states_i_dot = rocket_equations_of_motion(t,y)

global rho        % air density
global g       % force of gravity (earth fixed)
%global Cd        % drag coefficient
%global Cl        % lift coefficient
global area          % area matrix   [nose side side]
global m          % total mass
global I          % I matrix      [Ix Iy Iz]
%global r_cp2cg_b  % distance from cm to cg
global body_diam
global dcg       % distance from bottom to cg


% Values that need to be found experimentally
global Cm 
global CM 
global Cn 
global T % set thrust to 0
global CA 
global CY 
global CN 
global Lap
global Lay
global Lda
global delta
global dcgref

%% states
  states_i = y;
%  pos_i  = states_i(1:3);
  vel_i  = states_i(4:6);
  ang_i  = states_i(7:9); % (roll pitch yaw)
  angV_i = states_i(10:12);

%% Moment of Inertia Tenareaor
% I=[Ix 0 Ixz;...
%     0 Iy Iyz;...
%     Ixz Iyz Iz];

%% Calculate Angle of Attack
  yaw   = ang_i(3);
  pitch = ang_i(2);
  roll  = ang_i(1);
  
  alpha = atan2(vel_i(3),vel_i(1));
  beta  = asin(vel_i(1)/norm(vel_i));
    
  ap    = atan2(vel_i(3),vel_i(1));
  ay    = atan2(vel_i(2),vel_i(1));
  
  R=angV_i(3);
  Q=angV_i(2);
  P=angV_i(1);

%% Aerodynamic coefficients
%Cd =  1.28*sin(norm(AOA)); 
%Cl =  2*pi*alpha;
%Cy =  2*pi*beta;
q  =  0.5*rho*norm(vel_i)^2;  % dynamic pressure

Clap = Lap/(q*area*body_diam); % slope of rolling moment due to pitch
Clay = Lay/(q*area*body_diam); 
Clda = Lda/(q*area*body_diam);
%Cm= pitching moment coefficient
%CM= rate of change of pitching moment
%Cn= yawing moment coefficient
%CN= rate of change of yawing moment
%CY= side force coefficient
%dcg-dcgref= distance between real and referance center of gravity
                 
%% Moments due to Aerodynamics
L=q*area*body_diam*(Clap*ap+Clay*ay+Clda*delta);
M=q*area*body_diam*((Cm+CM*(Q*body_diam)./(2*norm(vel_i)))+CM*((dcg-dcgref)/body_diam));
N=q*area*body_diam*((Cn+CN*(R*body_diam)./(2*norm(vel_i)))+CY*((dcg-dcgref)/body_diam));
 
%% Forces due to Aerodynamics

%alpha=atan(vel_i(3)/vel_i(1));
%beta=atan(vel_i(2)/vel_i(1));

Fw = m*g*[-sin(pitch);
          sin(roll)*cos(pitch);
          cos(roll)*cos(pitch)]; % Total applied force on the rocket
Fa = -q *[CA+(alpha^2 + beta^2);
          CN*beta;
          CN*alpha            ]; % Total aerodynamic force

F = Fw;% + Fa

%% Equations of Motion
A=[cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);...
   cos(pitch)*cos(yaw) sin(roll)*sin(pitch)*cos(yaw)+cos(roll)*sin(yaw) cos(roll)*sin(pitch)*cos(yaw)*sin(roll)*sin(yaw);...
   -sin(pitch) sin(roll)*cos(pitch) cos(roll)*cos(pitch)];

B=[1 sin(roll)*tan(pitch) cos(roll)*tan(pitch);...
   0 cos(pitch)           -sin(roll)          ;...
   0 sin(roll)/cos(pitch) cos(roll)./cos(pitch)];

C=[0 -R  Q;...
   R  0 -P;...
  -Q  P  0];

pos_i_dot=A*vel_i; %body fixed velocity to ground fixed velocity

ang_i_dot=B*angV_i; %body fixed ang velocity to ground fixed ang velocity

vel_i_dot=(1/m)*F-C*vel_i; %Newton's law in non inertial body frame

angV_i_dot=I\([L;M;N]-C*I*angV_i); %Newton's law rotational in non inertial body frame 

states_i_dot = [pos_i_dot; vel_i_dot; ang_i_dot; angV_i_dot];