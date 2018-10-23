function states_i_dot = EquationsOfMotion(t,states,temp_thrustCurve,tspan,rocket)
global rho g

[d,time_index] = min(abs(t-tspan)); % This does the same thing as the find() command
numMotors = size(temp_thrustCurve); % size is the number of rocket motors

for i = 1:numMotors(2) % Finds the thrust value corresponding to current time
thrust(i) = temp_thrustCurve(i).thrust(time_index);
end

% External Forces (inertial frame)
Fg_i     = [0; 0; g*rocket.m]; % force of gravity in the Inertial Frame
thrust_b = [zeros(2,numMotors(2)); thrust];     % thrust in the Body Fixed Frame

for i = 1:numMotors(2)
thrust_i(:,i) = quatrotate(quatconj(circshift(states(7:10),1)'),thrust_b(:,i)')';
end
netThrust_i = sum(thrust_i,2); % sum of all the thrusts from each motor

Fnet_i = Fg_i + netThrust_i; % Net force in the Inertial Frame

% External Moments (inertial frame)
Mnet = [0; 0; 0]; % set to 0 for now, but will change when we include aerodynamics

% Omega Matrix to calculate qdot
omega = [[0            states(13) -states(12) states(11)];
         [-states(13)  0           states(11) states(12)];
         [ states(12) -states(11)  0          states(13)];
         [-states(11) -states(12) -states(13) 0         ]]; % still don't know what this is

% Define states_i_dot as the solutions to the equations of motion
posdot_i = states(4:6);               % inertial frame velocity
veldot_i = Fnet_i / rocket.m;         % inertial frame acceleration
qdot     = 0.5*omega*states(7:10);    % time derivative of quaternions
wdot     = [(rocket.I(2)-rocket.I(3))*states(12)*states(13)/rocket.I(1)+Mnet(1)/rocket.I(1);
            (rocket.I(3)-rocket.I(1))*states(10)*states(13)/rocket.I(2)+Mnet(2)/rocket.I(2);
            (rocket.I(1)-rocket.I(2))*states(10)*states(12)/rocket.I(3)+Mnet(3)/rocket.I(3)]; % angular acceleration

states_i_dot = [posdot_i; veldot_i; qdot; wdot];



