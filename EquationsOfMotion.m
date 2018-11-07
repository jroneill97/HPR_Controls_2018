function states_i_dot = EquationsOfMotion(states,rocket,thrust_b,fins)
%% Define the quaternions
    q           = states(7:10);
%% External Forces (inertial frame)
% Aerodynamic forces due to rocket body
    vel_i       = states(4:6);
    vel_b       = quaternion_I_to_B(q,vel_i);
    speed       = norm(vel_i);
    alpha       = atan2(vel_b(1),vel_b(3));
    beta        = asin(vel_b(2)/speed);
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
netThrust_i = quaternion_B_to_I(q,thrust_b); 
    
%% Net Force on the rocket body
Fnet_i      = Fexternal_i + netThrust_i;

%% External Moments (inertial frame)
% Rotate the distance from CP to CG into the inertial frame
    cp2cg_b     = rocket.dcp-rocket.dcg;
    cp2cg_i     = quaternion_B_to_I(q,cp2cg_b); % distance from the cp to cg in inertial frame
% Moment due to aerodynamic force of rocket body
Mad_i       = cross(cp2cg_i,Fad_i);
% Net External Moments
Mexternal_i = Mad_i;
    
%% Moments due to Fins
% Fin locations (body fixed)
    finLatOffset = 0.1; % Flipper offset from z-axis
    finCGOffset  = -rocket.dcg(3); % Flipper offset from the CG
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
    Mnet_i       = Mexternal_i + Mfins_i;
    
%% Omega Matrix and angular momentum to calculate qdot
    Omega       = [[0            states(13) -states(12) states(11)];
                   [-states(13)  0           states(11) states(12)];
                   [ states(12) -states(11)  0          states(13)];
                   [-states(11) -states(12) -states(13) 0         ]];
    H           =  (rocket.I)'.*states(11:13);                   % angular momentum

%% Define states_i_dot as the solutions to the equations of motion
posdot_i    = states(4:6);               % inertial frame velocity
veldot_i    = Fnet_i / rocket.m;         % inertial frame acceleration
qdot        = 0.5*Omega*q;    % time derivative of quaternions
wdot        = ((Mnet_i-cross(states(11:13),H))' ./ (rocket.I))'; % angular acceleration

%% Output term (dx)
states_i_dot    = [posdot_i; veldot_i; qdot; wdot];

%%
%%
%%
%%
%%
%%
%% Define all functions present here
%% ----------------------------------------------------------------
%% quaternion_I_to_B
function B = quaternion_I_to_B(q,A)
% Rotation from Body fixed to Inertial frame
QiB = dcm_from_q(q);
B = QiB*A;
end
%% quaternion_B_to_I
function I = quaternion_B_to_I(q,A)
% Rotation from Body fixed to Inertial frame
QbI = dcm_from_q(q);
I = QbI'*A;
end
%% dcm_from_q
function Q = dcm_from_q(q)
% ~~~~~~~~~~~~~~~~~~~~~~~
%{
  This function calculates the direction cosine matrix
  from the quaternion.

  q - quaternion (where q(4) is the scalar part)
  Q - direction cosine matrix
%}

q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);

Q = [q1^2-q2^2-q3^2+q4^2,      2*(q1*q2+q3*q4),       2*(q1*q3-q2*q4);
         2*(q1*q2-q3*q4), -q1^2+q2^2-q3^2+q4^2,       2*(q2*q3+q1*q4);
         2*(q1*q3+q2*q4),      2*(q2*q3-q1*q4),  -q1^2-q2^2+q3^2+q4^2 ];
end %dcm_from_q
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
end
