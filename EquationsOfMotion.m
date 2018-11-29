function states_i_dot = EquationsOfMotion(states,rocket,motorCluster,thrust,windSpeed,fins)
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
    Fad_v       = 0.5*(1.225)*(speed^2).*C_v;
    q_v         = angle2quat(0,-alpha,beta);
    Fad_b_quat  = quatrotate(quatconj(q_v),Fad_v')';
Fad_i           = quaternion_B_to_I(q,Fad_b_quat);
    
Fg_i            = [0;0;-9.8*rocket.m];
   
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
Fwind_i            = [0.5*1.225*(windSpeed^2)*rocket.Cla; 0; 0];
%% External force
Fexternal_i     = Fg_i + Fad_i + Fwind_i;

%% Thrust force
    [~, motorCount] = size(motorCluster);
    Tmotors     = [zeros(2,motorCount);thrust]; % Creates a column vector for each motor    
    Tnet_b      = sum(Tmotors,2);
    Tnet_i      = quaternion_B_to_I(q,Tnet_b);
%% Net Force on the rocket body
Fnet_i      = Fexternal_i + Tnet_i;

%% External Moments (inertial frame)
% Rotate the distance from CP to CG into the inertial frame
    cp2cg_b     = rocket.dcg-rocket.dcp;
    cp2cg_i     = quaternion_B_to_I(q,cp2cg_b); % distance from the cp to cg in inertial frame  
% Moment due to aerodynamic force of rocket body
Mad_i       = cross(cp2cg_i,Fad_i);
% Net External Moments
Mexternal_i = Mad_i;

%% Moments due to rocket motors
    thrustCGOffset  = rocket.L; % Motor offset from the CG
    for i = 1:motorCount
        Mmotors_b(:,i) = cross([motorCluster(i).location(1:2);thrustCGOffset],Tmotors(:,i));
    end
    Mmotors_net_b = sum(Mmotors_b,2);
Mmotors_net_i = quaternion_B_to_I(q,Mmotors_net_b);
%% Moments due to Fins
% Fin locations (body fixed)
    finLatOffset = 0.2;      % Flipper offset from z-axis
    finCGOffset  = (rocket.dcg(3) - rocket.L); % Flipper offset from the CG
    r1_b         = [finLatOffset;  0;            finCGOffset];
    r2_b         = [0;             finLatOffset; finCGOffset];
    r3_b         = [-finLatOffset; 0;            finCGOffset];
    r4_b         = [0;            -finLatOffset; finCGOffset];
% Fin forces
    F1_b         = [0       ; fins(1) ; 0];
    F2_b         = [-fins(2); 0       ; 0];
    F3_b         = [0       ; -fins(3); 0];
    F4_b         = [fins(4) ; 0       ; 0];
% body-fixed moments due to the flippers
    M1_b         = cross(r1_b,F1_b);
    M2_b         = cross(r2_b,F2_b);
    M3_b         = cross(r3_b,F3_b);
    M4_b         = cross(r4_b,F4_b);
    Mfins_b      = M1_b + M2_b + M3_b + M4_b;
% intertial-frame moments due to the flippers
Mfins_i      = quaternion_B_to_I(q,Mfins_b);
    
%% Net moments on the rocket body
    Mnet_i       = Mexternal_i + Mfins_i + Mmotors_net_i;
    Mnet_b       = quaternion_I_to_B(q,Mnet_i);
    w_b          = quaternion_I_to_B(q,states(11:13));
%% Omega Matrix and angular momentum to calculate qdot
    Omega       = [[0            states(13) -states(12) states(11)];
                   [-states(13)  0           states(11) states(12)];
                   [ states(12) -states(11)  0          states(13)];
                   [-states(11) -states(12) -states(13) 0         ]];
    H           =  (rocket.I)'.*states(11:13);                   % angular momentum

%% Define states_i_dot as the solutions to the equations of motion
posdot_i    = vel_i;                  % inertial frame velocity
veldot_i    = Fnet_i / rocket.m;         % inertial frame acceleration
q_shifted   = circshift(q,-1);           % need to move the scalar part back to the 4th index
qdot        = circshift(0.5*Omega*q_shifted,1);   % time derivative of quaternions
wdot        = ((Mnet_b-cross(w_b,H))' ./ (rocket.I))'; % angular acceleration

%% Output term (dx)
states_i_dot    = [posdot_i; veldot_i; qdot; wdot];

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% end of Equations of Motion
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%% quaternion_I_to_B
function B = quaternion_I_to_B(q,A)
% Rotation from Body fixed to Inertial frame
% QiB = dcm_from_q(circshift(q,-1)); %%%% Uncomment if not using Aerospace
% B = QiB*A;                         %%%% Toolbox. Delete circshift too
B = quatrotate(q',A')';
end
%% quaternion_B_to_I
function I = quaternion_B_to_I(q,A)
% Rotation from Body fixed to Inertial frame
% QbI = dcm_from_q(circshift(q,-1)); %%%% Uncomment if not using Aerospace
% I = QbI'*A  ;                      %%%% Toolbox. Delete circshift too
I = quatrotate(quatconj(q'),A')';
end
end
