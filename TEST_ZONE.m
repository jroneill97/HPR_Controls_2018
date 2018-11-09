 %% Define initial States
states = zeros(1,13); % initialize state matrix

initial_yaw   = 0.00;
initial_pitch = 0.00;
initial_roll  = 0.00;
states(7:10)  = quat_from_ypr(initial_yaw,initial_pitch,initial_roll); % Using ZYX for all rotations
states(11:13) = [0.0 0.0 0.0];
states(6)     = 50;
states(3)     = 0.0;
x0 = states';
%% Create the thrust Curve from the motorCluster structure
load motorCluster
t0       = 0;        % Initial Time
tf       = 10;       % Final Time
nip      = 2;        % Number of integration points
nsteps   = 500;
stepSize = tf/nsteps;
tspan    = [t0:stepSize:tf]';
motorCluster = CreateThrustCurves(motorCluster,tspan);
numMotors = size(motorCluster);
for j = 1:numMotors(2)
  thrust(:,j) = motorCluster(j).thrust;
end   
netThrust   = sum(thrust,2); % sum of all the thrusts from each motor






x = x0;

load rocket;
g    = -9.81;
rho  = 1.225;
m    = rocket.m;
Ix   = rocket.I(1);
Iy   = rocket.I(2);
Iz   = rocket.I(3);
S    = rocket.area;
Cd   = rocket.Cd;
Clda = rocket.Clda;
c    = rocket.dcg(3);
T    = netThrust(20)
u    = [1;2;3;4];

A      = zeros(13);

A(3,3) = (sqrt(2)/2)*sqrt(T/m - g - x(6)*x(6)*S*rho*Cd/m);
A(6,6) = (2*x(6)*rho*S*Cd)/m;
A(3,6) = (sqrt(2)*x(6)*rho*S*Cd)/sqrt(T*(m^2) - g*(m^3) - 0.5*(m^2)*rho*(x(6)^2)*S*Cd);
A(11,6)= Clda*c*0.5*(u(2)+u(4))*rho*S*x(6)/Ix;
A(12,6)= Clda*c*0.5*(u(1)+u(3))*rho*S*x(6)/Iy;
A(13,6)= Clda*c*0.25*(-u(1)-u(2)+u(3)+u(4))*rho*S*x(6)/Iz;


B = zeros(13,4);

B(11,2)=.25*Clda*rho*S*x(6)^2*c/Ix;
B(11,4)=.25*Clda*rho*S*x(6)^2*c/Ix;
B(12,1)=.25*Clda*rho*S*x(6)^2*c/Iy;
B(12,3)=.25*Clda*rho*S*x(6)^2*c/Iy;
B(13,1)=.125*Clda*rho*S*x(6)^2*c/Iz;
B(13,2)=.125*Clda*rho*S*x(6)^2*c/Iz;
B(13,3)=.125*Clda*rho*S*x(6)^2*c/Iz;
B(13,4)=.125*Clda*rho*S*x(6)^2*c/Iz;

C = zeros(13);
C(11,11) = 1;
C(12,12) = 1;
C(13,13) = 1;
D = zeros(13,4);


u = [0; 0; 0; 0];

Q = eye(13);
R = eye(4);


























