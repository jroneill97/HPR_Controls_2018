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

pos_i  = states_i(1:3);
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

AOA = acos(dot(vel_i,ang_i)/(((norm(vel_i)*norm(ang_i) == 0) + norm(vel_i)*norm(ang_i))));

%% External Forces (inertial frame)
T_i   = quatrotate(quatconj(quaternions),[0 0 T])';
Fd_i  = -.5*Cd*rho*(pi*body_radius^2)*norm(vel_i)*vel_i;

rcp_i = quatrotate(quatconj(quaternions),rcp_b')';
 V = cross(vel_i,[1;0;0]) + cross(vel_i,[0;1;0]) + cross(vel_i,[0;0;1]);
 V = V/norm(V);
Fl_i  = .5*Cl*rho*area*(norm(vel_i)^2)*cross(vel_i,rcp_i); 
Fg_i  = m*g_i;

Fnet_i = T_i + Fd_i + Fl_i + Fg_i;

%% External Moments (inertial frame)

I_i   = quatrotate(quatconj(quaternions),I_b')';

Md_i  = cross(rcp_i,Fd_i);
Ml_i  = cross(rcp_i,Fl_i);
Mt_i  = cross(rcp_i,T_i );
Mg_i  = cross(rcp_i,Fg_i);

Mnet_i =    (Md_i + Ml_i + Mt_i + Mg_i);

%% Create time derivative of state matrix
pos_i_dot  = vel_i; %A*vel_b;                 % inertial position
ang_i_dot  = angVel_i;                % inertial angular velocity
vel_i_dot  = Fnet_i/m  ; %C*vel_i;       % inertial acceleration (transport theorem)
angV_i_dot = inv(I_i)*Mnet_i; % inertial angular acceleration

states_i_dot = [pos_i_dot; vel_i_dot; ang_i_dot; angV_i_dot];









