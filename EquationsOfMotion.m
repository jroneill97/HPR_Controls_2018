function states_i_dot = EquationsOfMotion(t,states,thrust,tspan,rocket)
global rho g
[d,time_index] = min(abs(t-tspan));
thrust    = thrust(time_index); % Finds the thrust value corresponding to T


% External Forces (inertial frame)


% External Moments (inertial frame)
Mnet = [0 0 0];

% Euler Equations
omega = [[0            states(13) -states(12) states(11)];
         [-states(13)  0           states(11) states(12)];
         [ states(12) -states(11)  0          states(13)];
         [-states(11) -states(12) -states(13) 0         ]]; % still don't know what this is
     
H      = (rocket.I).*states(11:13)';                   % angular momentum

A = rocket.I(1);
B = rocket.I(2);
C = rocket.I(3);

wdot_i =  [(B-C)*states(12)*states(13)/A+Mnet(1)/A;
           (C-A)*states(10)*states(13)/B+Mnet(2)/B;
           (A-B)*states(10)*states(12)/C+Mnet(3)/C];

% Define states_i_dot as the solutions to the equations of motion
posdot_i = states(4:6);                       % position
veldot_i = [0;0;0];%(g*rocket.m)+thrust];         % inertial frame acceleration
qdot     = 0.5*omega*states(7:10);            % time derivative of quaternions
% wdot_i   = ((Mnet-cross(states(11:13),H)) ./ (rocket.I))'; % body fixed angular acceleration

states_i_dot = [posdot_i; veldot_i; qdot; wdot_i];









