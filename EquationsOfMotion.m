function states_i_dot = EquationsOfMotion(t,states,temp_thrustCurve,tspan,rocket)
global rho Fg_i

[d,time_index] = min(abs(t-tspan)); % This does the same thing as the find() command
numMotors = size(temp_thrustCurve); % size is the number of rocket motors

for i = 1:numMotors(2) % Finds the thrust value corresponding to current time
thrust(i) = temp_thrustCurve(i).thrust(time_index);
end

% External Forces (inertial frame)
thrust_b = [zeros(2,numMotors(2)); thrust];     % thrust in the Body Fixed Frame

for i = 1:numMotors(2)
thrust_i(:,i) = quatrotate(quatconj(circshift(states(7:10),1)'),thrust_b(:,i)')';
end
netThrust_i = sum(thrust_i,2); % sum of all the thrusts from each motor

Fnet_i = Fg_i + netThrust_i; % Net force in the Inertial Frame
%!!! Include the next part

% External Moments (inertial frame)
Mnet = [0; 0; 0]; % set to 0 for now, but will change when we include aerodynamics

% Omega Matrix and angular momentum to calculate qdot
Omega = [[0            states(13) -states(12) states(11)];
         [-states(13)  0           states(11) states(12)];
         [ states(12) -states(11)  0          states(13)];
         [-states(11) -states(12) -states(13) 0         ]];
H     = (rocket.I)'.*states(11:13);                   % angular momentum

% Define states_i_dot as the solutions to the equations of motion
posdot_i = states(4:6);               % inertial frame velocity
veldot_i = Fnet_i / rocket.m;         % inertial frame acceleration
qdot     = 0.5*Omega*states(7:10);    % time derivative of quaternions
wdot     = ((Mnet-cross(states(11:13),H))' ./ (rocket.I))'; % angular acceleration

states_i_dot = [posdot_i; veldot_i; qdot; wdot];



