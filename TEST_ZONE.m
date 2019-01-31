%% MQP work for Rocket Controller
clc; clear all; close all;

syms x y z u v w T m g rho Cl Cd A_fin A_front q1 q2 q3 q4 wx wy wz Mx My Mz Ixx Iyy Izz
% define matrix groups
q = [q1; q2; q3; q4];
w0 = [wx ;wy; wz];
M = [Mx; My; Mz];
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
H = [Ixx*wx; Iyy*wy; Izz*wz];
vel = (u^2+v^2+w^2);
% define matrix helper formulas
w_x = [0 -wz wy; wz 0 -wx; -wy wx 0];
O = [-w_x w0; -w0' 0];
q_dot = .5*O*q;
w_dot = I\(M-cross(w0,H));

%Aerodynamic Force/Moment
a = -atan(v/w);
b = asin(v/sqrt(vel));
T_bi = [2*(q2*q4+q1*q3)*T; 2*(q3*q4-q1*q2)*T; (q1^2-q2^2-q3^2+q4^2)*T];
F_ad = [.5*rho*sqrt(vel)*Cl*sin(a)*A_fin;...
        .5*rho*sqrt(vel)*Cl*sin(b)*A_fin;...
        .5*rho*sqrt(vel)*Cd*A_front]';
%DCM for velocity fixed to body fixed frame of reference
R_vb(1,:) = [1/sqrt(v^2/w^2+1) -v^2/(w*sqrt(v^2/w^2+1)*sqrt(vel))...
            v*(1-(v^2/sqrt(vel)))/(w*sqrt(v^2/w^2+1))];
R_vb(2,:) = [0 sqrt(1-v^2/vel) v/sqrt(vel)];...
R_vb(3,:) = [-v/(w*sqrt(v^2/w^2 + 1)) -v/(sqrt(v^2/w^2 + 1)*sqrt(vel))...
            (1-v^2/sqrt(vel))/sqrt(v^2/w^2 + 1)];
R_bi = [q1^2-q2^2-q3^2+q4^2 2*(q1*q2+q3*q4) 2*(q1*q3-q2*q4);...
        2*(q2*q1-q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3+q1*q4);...
        2*(q3*q1+q2*q4) 2*(q3*q2-q1*q4) -q1^2-q2^2+q3^2+q4^2];
F_aero = F_ad*R_vb*R_bi;
F = T_bi+[0;0;-m*g]+F_aero';
%disp(F);

x_dot = [u; v; w; F(1)/m; F(2)/m; F(3)/m; q_dot; w_dot]; %13x1 column vector
%disp(x_dot);

vars = [u; v; w; q1; q2; q3; q4; wx; wy; wz];
A = jacobian(x_dot,vars); %13x13 matrix
%disp(A);

C = [0 0 1 0 0 0 0 0 0 0 0 0 0;...
    0 0 0 0 0 0 0 0 0 0 1 0 0;...
    0 0 0 0 0 0 0 0 0 0 0 1 0;...
    0 0 0 0 0 0 0 0 0 0 0 0 1]*x_dot;
%disp(C);
