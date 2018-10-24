function states_i_dot = EquationsOfMotion(t,states,tspan,rocket,dynamicPressure,Fg_i,thrustCurves)
qScalarFirst = circshift(states(7:10),1);
q            = states(7:10);
%% External Forces (inertial frame)

% Thrust Forces
    [d,time_index] = min(abs(t-tspan)); % This does the same thing as the find() command
    for j = 1:7 % Finds the thrust value corresponding to current time
        thrust_b(j)     = thrustCurves(j).thrust(time_index);
    end   
    netThrust_b   = [0;0;sum(thrust_b,2)]; % sum of all the thrusts from each motor
    netThrust_i   = quatrotate(quatconj(qScalarFirst'),netThrust_b')'; 

% Aerodynamic forces
    vel_i = states(4:6);
    speed = norm(vel_i);
    
    alpha = atan2(vel_i(1),vel_i(3));
    beta  = atan2(vel_i(2),vel_i(3));
    
    Cl = 2*pi*alpha;
    
    Lalpha_v        = dynamicPressure*(speed^2)*Cl;
    Lbeta_v         = dynamicPressure*(speed^2)*Cl;
    D_v             = dynamicPressure*(speed^2)*rocket.Cd;
    
    Fad_v           = [Lbeta_v; Lalpha_v; -D_v];
    Rvel2body     = (R2(-alpha)*R1(beta))';
    Fad_b         = Rvel2body *Fad_v;
    Fad_i         = quatrotate(quatconj(qScalarFirst'),Fad_b')';
    
% Net External forces
    Fnet_i = Fg_i + netThrust_i + Fad_i; % Net force in the Inertial Frame

%% External Moments (inertial frame)
    rcp_i  = quatrotate(quatconj(qScalarFirst'),[0 0 -rocket.dcg])'; % distance from the cp to cg
    Mad_i  = cross(rcp_i,Fad_i);


    Mnet_i = Mad_i;
%% Omega Matrix and angular momentum to calculate qdot
    Omega = [[0            states(13) -states(12) states(11)];
             [-states(13)  0           states(11) states(12)];
             [ states(12) -states(11)  0          states(13)];
             [-states(11) -states(12) -states(13) 0         ]];
    H     =  (rocket.I)'.*states(11:13);                   % angular momentum

%% Define states_i_dot as the solutions to the equations of motion
    posdot_i = states(4:6);               % inertial frame velocity
    veldot_i = Fnet_i / rocket.m;         % inertial frame acceleration
    qdot     = 0.5*Omega*q;    % time derivative of quaternions
    wdot     = ((Mnet_i-cross(states(11:13),H))' ./ (rocket.I))'; % angular acceleration

states_i_dot = [posdot_i; veldot_i; qdot; wdot];



