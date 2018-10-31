% Linearize the equations of motion (need to do this every time the rocket
% parameters change!!!!)

load rocket
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 T Fx Fy fin1 fin2 fin3 fin4

states   = [x1 ;x2; x3; x4 ;x5 ;x6; x7 ;x8 ;x9; x10; x11; x12; x13];
thrust_b = [0;0;T];
dFins    = [fin1;fin2;fin3;fin4];

xDot     = EquationsOfMotion(states,rocket,thrust_b,Fx,Fy,dFins);

diary jacobianMatrix
A        = jacobian(xDot) % <<<< A is the linearized matrix
% Now, open the jacobianMatrix file in Matlab and you'll see the matrix
% there
diary off