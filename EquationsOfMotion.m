function states_i_dot = EquationsOfMotion(states,rocket,motorCluster,thrust,fins)
%% Define the quaternions
    q           = states(7:10);
%% External Forces (inertial frame)
% Aerodynamic forces due to rocket body
    vel_i       = states(4:6);
    vel_b       = quaternion_I_to_B(q,vel_i);
    speed       = norm(vel_b);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    z_b = quaternion_I_to_B(q,[0;0;1]);
%     u = cross(vel_b,z_b)/norm(cross(vel_b,z_b));
%     a = acos(dot(vel_b,z_b)/(norm(vel_b)*norm(z_b)));
%     vel_q = [sin(a/2)*u(1), sin(a/2)*u(2), sin(a/2)*u(3), cos(a/2)];
% 
%     [a,b,c] = quat2angle(vel_q,'ZYX')
      alpha       = atan2(vel_b(1),vel_b(3));
      beta        = asin(vel_b(2)/speed);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Cx_v        = rocket.Cla * (alpha);
    Cy_v        = rocket.Clb * (beta);
    Cz_v        = 0.05*rocket.Cd0; % Estimating the drag to be directly opposite the nose
    C_v         = [-Cx_v; -Cy_v; -Cz_v];
    Fad_v       = 0.5*(1.225)*rocket.area*(speed^2).*C_v;
    Rvel2body   = (R2(-alpha)*R3(beta))';
    Fad_b       = Rvel2body *Fad_v;
Fad_i       = quaternion_B_to_I(q,Fad_b);
    
Fg_i        = [0;0;-9.8*rocket.m];
    
Fexternal_i = Fg_i +Fad_i;
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
    thrustCGOffset  = (rocket.dcg(3) - rocket.L); % Motor offset from the CG
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
    F1_b         = [0       ; fins(1) ;0];
    F2_b         = [-fins(2); 0       ;0];
    F3_b         = [0       ; -fins(3);0];
    F4_b         = [fins(4) ; 0       ;0];
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
    xyz_b        = quaternion_I_to_B(q,states(11:13));
%% Omega Matrix and angular momentum to calculate qdot
    Omega       = [[0            states(13) -states(12) states(11)];
                   [-states(13)  0           states(11) states(12)];
                   [ states(12) -states(11)  0          states(13)];
                   [-states(11) -states(12) -states(13) 0         ]];
    H           =  (rocket.I)'.*states(11:13);                   % angular momentum

%% Define states_i_dot as the solutions to the equations of motion
posdot_i    = states(4:6);               % inertial frame velocity
veldot_i    = Fnet_i / rocket.m;         % inertial frame acceleration
q_shifted   = circshift(q,-1);           % need to move the scalar part back to the 4th index
qdot        = circshift(0.5*Omega*q_shifted,1);   % time derivative of quaternions
wdot        = ((Mnet_i-cross(states(11:13),H))' ./ (rocket.I))'; % angular acceleration

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
