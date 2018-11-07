function states_i_dot = LinearizedEquationsOfMotion(states,rocket,thrust_b,fins)
g = -9.8;
rho = 1.225;
T = thrust_b;
x = states;
u = fins;

%% A matrix
A =...
[[ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0, (2^(1/2)*(rocket.m*g + T - (rocket.Cd*rocket.area*rho*x(6)^2)/2)^(1/2))/(2*(rocket.m*x(3))^(1/2)),                                                                                    0, 0, -(2^(1/2)*rocket.Cd*rocket.area*rho*x(3)*x(6))/(2*rocket.m*((x(3)*(-(rocket.Cd*rocket.area*rho*x(6)^2)/2 + rocket.m*g + T))/rocket.m)^(1/2)), 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                -(rocket.Cd*rocket.area*rho*x(6))/rocket.m, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                                                                                                  0, 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                    (rocket.Clda*rocket.area*(rocket.dcg(3))*rho*x(6)^2*(u(2)/2 + u(4)/2))/(2*rocket.I(1)), 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                                    (rocket.Clda*rocket.area*(rocket.dcg(3))*rho*x(6)^2*(u(1)/2 + u(3)/2))/(2*rocket.I(2)), 0, 0, 0, 0, 0, 0, 0];
 [ 0, 0,                                                             0, 0, 0,                 -(rocket.Clda*rocket.area*(rocket.dcg(3))*rho*x(6)^2*(u(1)/4 + u(2)/4 - u(3)/4 - u(4)/4))/(2*rocket.I(3)), 0, 0, 0, 0, 0, 0, 0]];

%% Calculate derivative about equilibrium point
states_i_dot = A*states;
