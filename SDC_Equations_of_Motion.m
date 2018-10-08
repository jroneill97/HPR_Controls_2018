function states_i_dot = rocket_equations_of_motion(yaw,states_i)

global rho        % air density
global g_i       % force of gravity (earth fixed)
%global Cd        % drag coefficient
%global Cl        % lift coefficient
global A          % area matrix   [nose side side]
global m          % total mass
global I          % I matrix      [Ix Iy Iz]
global r_cp2cg_b  % distance from cm to cg
Fg_i   = m*g_i;
%% states
  pos_i  = states_i(1:3);
  vel_i  = states_i(4:6);
  ang_i  = states_i(7:9); % earth-fixed -> (roll pitch yaw)(phi theta psi)
  angV_i = states_i(10:12);
%% Calculate Angle of Attack
  yaw   = pos_i(3);
  pitch = pos_i(2);
  roll  = pos_i(1);
  
  alpha = atan2(vel_i(3)/vel_i(1));
  beta  = asin(vel_i(1)/norm(vel_i));
  AOA   = [alpha; beta];
  
  ap    = atan2(vel_i(3)/vel_i(1));
  ay    = atan2(vel_i(2)/vel_i(1));

%% Aerodynamic coefficients
Cd =  1.28*sin(norm(AOA));
Cl =  2*pi*alpha;
Cy =  2*pi*beta;
q  =  0.5*rho*norm(vel_i)^2;  % dynamic pressure

l = diam
b = wingspan
c = length of rocket

Lap = .2;
Lay = .2;
Lda = .2;

Clap = Lap/(q*S*diam); % slope of rolling moment due to pitch
Clay = lay/(q*S*diam); 
Clda = Lda/(q*S*diam);
%% Translational accelerations due to aerodynamics
Ax = (T*cos(alpha)*cos(beta)-Cd*q*S)/Fg_i;
Ay = (T*cos(alpha)*sin(beta)+Cy*q*S)/Fg_i;
Az = (T*sin(alpha)+Cl*q*S)          /Fg_i;

xDD = (Ax*cos(pitch)*cos(yaw)-Ay*sin(yaw)-Az*sin(pitch)*cos(yaw))*g_i;
yDD = (Ax*cos(pitch)*sin(yaw)+Ay*cos(yaw)-Az*sin(pitch)*sin(yaw))*g_i;
zDD = (1-Ax*sin(pitch)-Az*cos(pitch))                            *g_i;

%% Rotational accelerations due to aerodynamics
def_roll = 0;
rollDD   = Q*R*((Iy-Iz)/Ix)+(q*S*diam/Ix)*(Clap*ap+Clay*ay+Clda*def_roll);
pitchDD  = ((Iz-Ix)/Iy)*P*R+(q*S*diam/Iy)*(C
yawDD    = 



%% Solving for state time derivatives
pos_i_dot  = vel_i;
vel_i_dot  = [xDD;yDD;zDD];  % might not be right
ang_i_dot  = angV_i;
angV_i_dot = Tcp_i./I;   % might not be right

states_i_dot = [pos_i_dot; vel_i_dot; ang_i_dot; angV_i_dot];