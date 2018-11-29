function states_i_dot = MainChuteEquationsOfMotion(states,rocket,mainChute,windSpeed)
%% Define the quaternions
    q           = states(7:10);
%% External Forces (inertial frame)
%  Aerodynamic forces due to wind
    vel_i       = states(4:6);
    vel_b       = quaternion_I_to_B(q,vel_i);
    lateralSpeed       = norm(vel_i(1:2));
%     alpha       = -atan2(vel_b(1),vel_b(3)); % Angle of attack
%     beta        = -asin(vel_b(2)/lateralSpeed);     % Side-slip
    FadX_i       = -0.5*(1.225)*(vel_i(1)^2)*rocket.area*rocket.Cla;
    FadY_i       = -0.5*(1.225)*(vel_i(2)^2)*rocket.area*rocket.Clb;
    FmainChute_i = 0.5*(1.225) *(vel_i(3)^2)*mainChute.A*mainChute.Cd; % Calculates drag induced on rocket body from the main chute

Fad_i           = [FadX_i; FadY_i; FmainChute_i]; % Estimating the aerodynamic forces
                                                  % to be along axis of
                                                  % inertial frame
Fg_i            = [0;0;-9.8*rocket.m];

Fwind_i            = [0.5*1.225*(windSpeed^2)*rocket.Cla; 0; 0];  

Fnet_i     = Fg_i + Fad_i + Fwind_i;

%% External Moments (inertial frame)
% Rotate the distance from CP to CG into the inertial frame
    cp2cg_b     = rocket.dcg-rocket.dcp;
    cp2cg_i     = quaternion_B_to_I(q,cp2cg_b); % distance from the cp to cg in inertial frame  
    dmainChute_i       = quaternion_B_to_I(q,-(rocket.dcg + [0;0;2]));
%% Moment due to main chute
    Mnet_i    = [0;0;0];%cross(dmainChute_i,Fnet_i); % Aproximates the chute to be situated at the CG plus 2 meters for the shock cord length estimate

%% Net moments on the rocket body
    Mnet_b      = quaternion_I_to_B(q,Mnet_i);
    w_b         = quaternion_I_to_B(q,states(11:13));
%% Omega Matrix and angular momentum to calculate qdot
    Omega       = [[0            states(13) -states(12) states(11)];
                   [-states(13)  0           states(11) states(12)];
                   [ states(12) -states(11)  0          states(13)];
                   [-states(11) -states(12) -states(13) 0         ]];
    H           =  (rocket.I)'.*states(11:13);                   % angular momentum

%% Define states_i_dot as the solutions to the equations of motion
posdot_i    = vel_i;               % inertial frame velocity
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