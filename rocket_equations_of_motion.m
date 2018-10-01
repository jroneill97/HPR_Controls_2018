function states_i_dot = rocket_equations_of_motion(t,states_i)

global rho        % air density
global g_i       % force of gravity (earth fixed)
%global Cd        % drag coefficient
%global Cl        % lift coefficient
global A          % area matrix   [nose side side]
global m          % total mass
global I          % I matrix      [Ix Iy Iz]
global r_cp2cg_b  % distance from cm to cg

%% states
% pos_i  = states_i(1:3);
  vel_i  = states_i(4:6);
  ang_i  = states_i(7:9); % earth-fixed -> (roll pitch yaw)(phi theta psi)
  angV_i = states_i(10:12);
%% Calculate Angle of Attack
  vel_i_angle = acos([(dot(vel_i,[1 0 0]')/norm(vel_i)); 
                      (dot(vel_i,[0 1 0]')/norm(vel_i));
                      (dot(vel_i,[0 0 1]')/norm(vel_i)) ]);
  AOA = vel_i_angle + ang_i; % adjusts the initial angles to 0,0,0
   %disp(rad2deg(AOA));
%% calculate the rotation matrix ReB (earth-fixed to body)
RiB =  angle2dcm(ang_i(3),ang_i(2),ang_i(1)); % R1(ang_i(3))*R2(ang_i(2))*R3(ang_i(1))

%% Aerodynamic forces (earth fixed)
Cl =  2*pi*(AOA);
Cd = .2*sin(AOA);
q  = .5*rho*norm(vel_i)^2;  % dynamic pressure
al_i = (Cl.*A*q)/m;
ad_i = (Cd.*A*q)/m;
a_ad_i  = al_i+ad_i;

%% Calculate body-fixed force vectors
g_b  = RiB * g_i;
a_ad_b = RiB * ad_i;

%% Calculate net force and torque on the rocket
anet_b = (g_b + a_ad_b);              % Net force
Tcp_b  = cross(r_cp2cg_b,a_ad_b);    % Net torque about the center of gravity

anet_i = RiB'*anet_b + cross(angV_i,vel_i);
Tcp_i  = RiB' * Tcp_b;

%% Solving for state time derivatives
pos_i_dot  = vel_i;
vel_i_dot  = anet_i;  % might not be right
ang_i_dot  = angV_i;
angV_i_dot = Tcp_i./I;   % might not be right

states_i_dot = [pos_i_dot; vel_i_dot; ang_i_dot; angV_i_dot];