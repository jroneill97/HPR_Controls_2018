function states_i_dot = EquationsOfMotion(t,states,tspan,rocket,thrustCurves,Fx,Fy)
global aeroConstant Fg_i
qScalarFirst = circshift(states(7:10),1);
q            = states(7:10);
%% External Forces (inertial frame)

% Thrust Forces
    [d,time_index] = min(abs(t-tspan)); % This does the same thing as the find() command
    for j = 1:7 % Finds the thrust value corresponding to current time
      thrust_b(j) = thrustCurves(j).thrust(time_index);
    end   
    netThrust_b   = [0;0;sum(thrust_b,2)]; % sum of all the thrusts from each motor
    netThrust_i   = quatrotate(quatconj(qScalarFirst'),netThrust_b')'; 
% Force due to wind
    Fwind_i    = 50*[Fx; Fy; 0];
    
% Aerodynamic forces due to rocket body
    vel_i      = states(4:6);
    vel_b      = quatrotate(qScalarFirst',vel_i')';
    speed      = norm(vel_i);

    alpha  = atan2(vel_b(1),vel_b(3));
    beta   = asin(vel_b(2)/speed);
    Cx_v       = rocket.Cla * (alpha);
    Cy_v       = rocket.Clb * (beta);
    Cz_v       = .1*rocket.Cd0 * cos(.5*(alpha+beta));
    C_v        = [-Cx_v; -Cy_v; -Cz_v];
    
    Fad_v      = aeroConstant*(speed^2).*C_v;
    
    q_ab = angle2quat(-alpha,beta,0,'YZY');
    Fad_b = quatrotate(quatconj(q_ab),Fad_v')';
% %     
%      Rvel2body  = (R2(-alpha)*R3(beta))';
%      Fad_b      = Rvel2body * Fad_v
    Fad_i      = quatrotate(quatconj(qScalarFirst'),Fad_b')';
    
% Aerodynamic forces due to fin deflection
    

% Net External forces
    Fnet_i     = Fg_i + netThrust_i + Fad_i + Fwind_i; % Net force in the Inertial Frame

%% External Moments (inertial frame)

% Moment due to aerodynamic force of rocket body
    cp2cg_b    = rocket.dcp-rocket.dcg;
    cp2cg_i    = quatrotate(quatconj(qScalarFirst'),cp2cg_b')'; % distance from the cp to cg in inertial frame
    Mad_i      = cross(cp2cg_i,Fad_i);
    
% Moments due to wind
    Mwind_i    = cross(cp2cg_i,Fwind_i);

    Mnet_i     = Mad_i + Mwind_i;
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



