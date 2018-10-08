function states_i_dot = rocket_equations_of_motion(t,y)

%% define global variables
global rho        % air density
global g_i        % force of gravity (earth fixed)

global T          
global body_radius
global area

global m          % total mass
global I_b        % moment of inertia tensor in body fixed frame
global Cd Cl
global rcp_b


%% inertial states
states_i = y;

%pos_i  = states_i(1:3);
vel_i  = states_i(4:6);
ang_i  = states_i(7:9); % (roll pitch yaw)
angVel_i = states_i(10:12);

%% Calculate Angle of Attack
yaw   = ang_i(3);
pitch = ang_i(2);
roll  = ang_i(1); % rotating around length of rocket

quaternions = angle2quat(yaw,pitch,roll); % Quaternion vector

alpha = atan2(vel_i(3),vel_i(1));
beta  = asin(vel_i(3)/norm(vel_i));
z_b = quatrotate(quaternions,[0 0 1])';
AOA = subspace(vel_i,z_b);

%% External Forces (inertial frame)
T_i   = quatrotate(quatconj(quaternions),[0 0 T])';
% % % Fd_i  = -.5*Cd*sin(AOA)*rho*(pi*body_radius^2)*norm(vel_i)*vel_i
% % % 
% % % %z_b = quatrotate(quaternions,[0 0 1])';
% % % unitVel_i = vel_i/norm(vel_i);
% % % vel_i_norm = cross(unitVel_i,cross(unitVel_i,[0;0;1]));
% % % vel_i_norm_unit = vel_i_norm/(norm(vel_i_norm) + (norm(vel_i_norm) == 0));
% % % 
% % % Fl_i  = .5*Cl*(2*pi*sin(AOA))*rho*area*(norm(vel_i)^2)*vel_i_norm_unit;

unitv = null(vel_i(:).');
vel_b  = quatrotate(quaternions,vel_i')';
Fl_i = .5*Cl*(2*pi*sin(alpha))*rho*area*norm(vel_b)*unitv(:,1);
Fd_i = -.5*Cd*sin(AOA)*rho*(pi*body_radius^2)*norm(vel_b)*vel_i;

Fg_i  = m*g_i;

Fnet_i = T_i + + Fg_i + Fd_i + Fl_i + Fg_i;

%% External Moments (inertial frame)

rcp_i = quatrotate(quatconj(quaternions),rcp_b')';
I_i   = quatrotate(quatconj(quaternions),I_b')';

Md_i  = cross(rcp_i,Fd_i);
Ml_i  = cross(rcp_i,Fl_i);
Mt_i  = cross(rcp_i,T_i );
Mg_i  = cross([0;0;0],Fg_i);

Mnet_i =    Mt_i + Ml_i + Md_i + Mg_i;

%% Create time derivative of state matrix

% xdot = [(I_i(2)-I_i(3))*angVel_i(2)*angVel_i(3)/I_i(1)+Mnet_i(1)/I_i(1);
%         (I_i(3)-I_i(1))*angVel_i(1)*angVel_i(3)/I_i(2)+Mnet_i(2)/I_i(2);
%         (I_i(1)-I_i(2))*angVel_i(1)*angVel_i(2)/I_i(3)+Mnet_i(3)/I_i(3);
%       (sin(x(6))*angVel_i(1)+cos(x(6))*angVel_i(2))/sin(x(5));
%       (cos(x(6))*sin(x(5))*angVel_i(1)-sin(x(6))*sin(x(5))*angVel_i(2))/sin(x(5));
%       (-sin(x(6))*cos(x(5))*angVel_i(1)-cos(x(6))*cos(x(5))*angVel_i(2)+sin(x(5))*angVel_i(3))/sin(x(5))]
% 
omega = [0 angVel_i(3) -angVel_i(2) angVel_i(1);
         -angVel_i(3) 0 angVel_i(1) angVel_i(2);
         angVel_i(2) -angVel_i(1) 0 angVel_i(3);
         -angVel_i(1) -angVel_i(2) -angVel_i(3) 0];
pos_i_dot  = vel_i; %A*vel_b;                 % inertial position
ang_i_dot  = angVel_i;                % inertial angular velocity
vel_i_dot  = Fnet_i/m  ; %C*vel_i;       % inertial acceleration (transport theorem)
angV_i_dot = I_i \ Mnet_i; % inertial angular acceleration

states_i_dot = [pos_i_dot; vel_i_dot; ang_i_dot; angV_i_dot];









