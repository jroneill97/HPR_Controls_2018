function states_i_dot = equations_of_motion(states,rocket,motorCluster,thrust,windSpeed,u)
%% Define the quaternions
q           = states(7:10);
%% External Forces (inertial frame)
% Aerodynamic forces on rocket body
vel_i       = states(4:6);
vel_b       = quaternion_I_to_B(q,vel_i);
speed       = norm(vel_b);
alpha       = -atan2(vel_b(1),vel_b(3)); % Angle of attack
beta        = -asin(vel_b(2)/speed);     % Side-slip

Cx_v        = rocket.Cla * sin(alpha) * rocket.area;
Cy_v        = rocket.Clb * sin(beta)  * rocket.area;
Cz_v        = rocket.Cd  * rocket.frontArea; % Estimating the drag to be directly opposite the nose
C_v         = [Cx_v; Cy_v; -Cz_v];
F_ad_v      = 0.5*(1.225)*(speed^2).*C_v;
q_v         = angle2quat(0,-alpha,beta);
F_ad_b_quat = quatrotate(quatconj(q_v),F_ad_v')';
F_ad_i          = quaternion_B_to_I(q,F_ad_b_quat);

F_g_i           = [0;0;-9.8*rocket.m];

%% Wind Force
%     windVel_b   = quaternion_I_to_B(q,[windSpeed;0;0]);
%     alpha_wind  = -atan2(windVel_b(1),windVel_b(3)); % Angle of attack
%     beta_wind   = -asin(windVel_b(2)/windSpeed);     % Side-slip
%
%     Fx_wind        = -0.5*(1.225)*(windSpeed^2)*rocket.Cla * rocket.area * sin(alpha_wind);
%     Fy_wind        = -0.5*(1.225)*(windSpeed^2)*rocket.Clb * rocket.area * sin(beta_wind);
%     Fz_wind        = -0.5*(1.225)*(windSpeed^2)*rocket.Cd * rocket.frontArea;
%     Fwind_v        = [Fx_wind; Fy_wind; Fz_wind];
%     q_v            = angle2quat(0,-alpha_wind,beta_wind);
%     Fwind_b_quat   = quatrotate(quatconj(q_v),Fwind_v')';
F_wind_i            = [0.5*1.225*(windSpeed^2)*rocket.Cla; 0; 0];
%% External force
F_external_i     = F_g_i + F_ad_i + F_wind_i;

%% Thrust force
[~, motorCount] = size(motorCluster);
T_motors     = [zeros(2,motorCount);thrust]; % Creates a column vector for each motor
T_net_b      = sum(T_motors,2);
T_net_i      = quaternion_B_to_I(q,T_net_b);
%% Net Force on the rocket body
F_net_i      = F_external_i + T_net_i;

%% External Moments (inertial frame)
% Rotate the distance from CP to CG into the inertial frame
cp2cg_b     = rocket.dcp-rocket.dcg;
cp2cg_i     = quaternion_B_to_I(q,cp2cg_b); % distance from the cp to cg in inertial frame
% Moment due to aerodynamic force of rocket body
M_ad_i       = cross(cp2cg_i,F_ad_i);

%% Moments due to rocket motors
thrustCGOffset  = rocket.L; % Motor offset from the CG
for i = 1:motorCount
    M_motors_b(:,i) = cross([motorCluster(i).location(1:2);thrustCGOffset],T_motors(:,i));
end
Mmotors_net_b = sum(M_motors_b,2);
M_motors_net_i = quaternion_B_to_I(q,Mmotors_net_b);
%% Moments due to Fins
% Fin locations (body fixed)
d = 0.2159;                      % Flipper offset from z-axis
l  = -(rocket.dcg(3) + rocket.L); % Flipper offset from the CG
f1 = 0.5*(1.225)*rocket.A_rudder*speed^2*sign(u(1))*2*pi*min(abs(u(1)),deg2rad(30));
f2 = 0.5*(1.225)*rocket.A_rudder*speed^2*sign(u(2))*2*pi*min(abs(u(2)),deg2rad(30));
f3 = 0.5*(1.225)*rocket.A_rudder*speed^2*sign(u(3))*2*pi*min(abs(u(3)),deg2rad(30));
f4 = 0.5*(1.225)*rocket.A_rudder*speed^2*sign(u(4))*2*pi*min(abs(u(4)),deg2rad(30));
% %
r1_b = [ d  0   l];
r2_b = [ 0  d   l];
r3_b = [-d  0   l];
r4_b = [ 0 -d   l];
 
f1_b = [ 0   f1  0];
f2_b = [-f2   0  0];
f3_b = [ 0   -f3 0];
f4_b = [ f4  0   0];

M1_rudder_b =  cross(f1_b,r1_b);
M2_rudder_b =  cross(f2_b,r2_b);
M3_rudder_b =  cross(f3_b,r3_b);
M4_rudder_b =  cross(f4_b,r4_b);

%  M_rudder_b = (M1_rudder_b + M2_rudder_b + M3_rudder_b + M4_rudder_b)'
M_rudder_b = [l*(f1-f3); l*(f2-f4); -d*(f1+f2+f3+f4)];
% intertial-frame moments due to the flippers
M_rudder_i      = quaternion_B_to_I(q,M_rudder_b);

%% Net moments on the rocket body
M_net_i       = M_ad_i + M_rudder_i + M_motors_net_i;
M_net_b       = quaternion_I_to_B(q,M_net_i);        % Body-fixed net moments
w_b          = quaternion_I_to_B(q,states(11:13)); % Body-fixed angular velocity
%% Omega Matrix and angular momentum to calculate qdot
Omega       = [[0            states(13) -states(12) states(11)];
    [-states(13)  0           states(11) states(12)];
    [ states(12) -states(11)  0          states(13)];
    [-states(11) -states(12) -states(13) 0         ]];
H           =  (rocket.I)'.*states(11:13);                   % angular momentum

%% Define states_i_dot as the solutions to the equations of motion
posdot_i    = vel_i;                  % inertial frame velocity
veldot_i    = F_net_i / rocket.m;         % inertial frame acceleration
q_shifted   = circshift(q,-1);           % need to move the scalar part back to the 4th index
q_dot       = circshift(0.5*Omega*q_shifted,1);   % time derivative of quaternions
w_dot       = ((M_net_i-cross(states(11:13),H))' ./ (rocket.I))'; % angular acceleration

%% Output term (dx)
states_i_dot    = [posdot_i; veldot_i; q_dot; w_dot];

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% end of Equations of Motion
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%% quaternion_I_to_B
    function B = quaternion_I_to_B(q,A)
        % Rotation from Inertial fixed to Body frame
        B = quatrotate(q',A')';
    end
%% quaternion_B_to_I
    function I = quaternion_B_to_I(q,A)
        % Rotation from Body fixed to Inertial frame
        I = quatrotate(quatconj(q'),A')';
    end
end
