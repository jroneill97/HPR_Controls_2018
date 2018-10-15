function states_i_dot = EquationsOfMotion(t,states,thrust,rocket)
global rho g

% External Forces (inertial frame)


% External Moments (inertial frame)


% Define states_i_dot as the solutions to the equations of motion
pos_i_dot    = [thrust;thrust;thrust]; % inertial position
vel_i_dot    = zeros(3,1); % inertial acceleration (transport theorerocket.m)
angPos_i_dot = zeros(4,1); % inertial angular velocity
angVel_i_dot = zeros(4,1); % inertial angular acceleration

states_i_dot = [pos_i_dot; vel_i_dot; angPos_i_dot; angVel_i_dot];









