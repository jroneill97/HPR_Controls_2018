function states_i_dot = main_chute_equations_of_motion(states,rocket,mainChute)
%% Define the quaternions
    q           = states(7:10);
%% External Forces (inertial frame)
%  Aerodynamic forces due to wind
    vel_i       = states(4:6);
    FadX_i       = -0.5*(1.225)*sign(vel_i(1))*(vel_i(1)^2)*rocket.area*rocket.Cla;
    FadY_i       = -0.5*(1.225)*sign(vel_i(2))*(vel_i(2)^2)*rocket.area*rocket.Clb;
    FmainChute_i =  0.5*(1.225)*(vel_i(3)^2)*mainChute.A*mainChute.Cd; % Calculates drag induced on rocket body from the main chute

Fad_i           = [FadX_i; FadY_i; FmainChute_i]; % Estimating the aerodynamic forces
                                                  % to be along axis of
                                                  % inertial frame
Fg_i            = [0;0;-9.8*rocket.m];

Fwind_i            = [0;0;0]; %[0.5*1.225*(windSpeed^2)*rocket.Cla; 0; 0];  

Fnet_i     = Fg_i + Fad_i + Fwind_i;

%% Omega Matrix and angular momentum to calculate qdot
    Omega       = [[0            states(13) -states(12) states(11)];
                   [-states(13)  0           states(11) states(12)];
                   [ states(12) -states(11)  0          states(13)];
                   [-states(11) -states(12) -states(13) 0         ]];
%% Define states_i_dot as the solutions to the equations of motion
posdot_i    = vel_i;               % inertial frame velocity
veldot_i    = Fnet_i / rocket.m;         % inertial frame acceleration
q_shifted   = circshift(q,-1);           % need to move the scalar part back to the 4th index
qdot        = circshift(0.5*Omega*q_shifted,1);   % time derivative of quaternions
wdot        = -(1)*states(11:13); % angular acceleration

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