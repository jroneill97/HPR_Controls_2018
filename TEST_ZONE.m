%% MQP work for Rocket Controller
clc; clear all; close all;

syms x y z u v w T m g rho Cl Cd A_side A_fin A_front q1 q2 q3 q4 wx wy wz Mx My Mz Ixx Iyy Izz d l dcg dcp u1 u2 u3 u4
% define matrix groups
q = [q1; q2; q3; q4];
w0 = [wx ;wy; wz];
speed = (u^2+v^2+w^2)^2;

f1 = 0.5*rho*A_fin*u1*speed^2;
f2 = 0.5*rho*A_fin*u2*speed^2;
f3 = 0.5*rho*A_fin*u3*speed^2;
f4 = 0.5*rho*A_fin*u4*speed^2;

M_fins_b = [l*(f3-f1) l*(f4-f2) d*(f1+f2+f3+f4)];

% M_fins=[[l*(f2-f4)*(2*q1*q4-2*q2*q3)+d*(2*q1*q3+2*q2*q4)*(f1+f2+f3+f4)-l*(f1-f3)*(q1^2+q2^2-q3^2-q4^2)] ...
%  [- l*(f1-f3)*(2*q1*q4+2*q2*q3)-d*(2*q1*q2-2*q3*q4)*(f1+f2+f3+f4)- l*(f2 - f4)*(q1^2 - q2^2 + q3^2 - q4^2)] ...
%  [  d*(q1^2 - q2^2 - q3^2 + q4^2)*(f1 + f2 + f3 + f4) + l*(f1 - f3)*(2*q1*q3 - 2*q2*q4) - l*(f2 - f4)*(2*q1*q2 + 2*q3*q4)]];
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
H = [Ixx*wx; Iyy*wy; Izz*wz];

% define matrix helper formulas
w_x = [0 -wz wy; wz 0 -wx; -wy wx 0];
O = [-w_x w0; -w0' 0];
q_dot = 0.5*O*q;

%Aerodynamic Force/Moment
a = -atan(v/w);
b = asin(v/sqrt(speed));
T_bi = [2*(q2*q4+q1*q3)*T; 2*(q3*q4-q1*q2)*T; (q1^2-q2^2-q3^2+q4^2)*T];
F_ad = [0.5*rho*speed*Cl*A_side*((v/w)/sqrt((v^2/w)+1))*2*pi*atan(v/w);...
        0.5*rho*speed*Cl*A_side*(v/speed)*2*pi*atan(v/w);...
        0.5*rho*speed^2*Cd*A_front]';

%DCM for velocity fixed to body fixed frame of reference
R_vb(1,:) = [1/sqrt(v^2/w^2+1) -v^2/(w*sqrt(v^2/w^2+1)*sqrt(speed))...
            v*(1-(v^2/sqrt(speed)))/(w*sqrt(v^2/w^2+1))];
R_vb(2,:) = [0 sqrt(1-v^2/speed) v/sqrt(speed)];...
R_vb(3,:) = [-v/(w*sqrt(v^2/w^2 + 1)) -v/(sqrt(v^2/w^2 + 1)*sqrt(speed))...
            (1-v^2/sqrt(speed))/sqrt(v^2/w^2 + 1)];
R_bi = [q1^2-q2^2-q3^2+q4^2 2*(q1*q2+q3*q4) 2*(q1*q3-q2*q4);...
        2*(q2*q1-q3*q4) -q1^2+q2^2-q3^2+q4^2 2*(q2*q3+q1*q4);...
        2*(q3*q1+q2*q4) 2*(q3*q2-q1*q4) -q1^2-q2^2+q3^2+q4^2];
F_aero = F_ad*R_vb*R_bi;
F = T_bi+[0;0;-m*g]+[F_aero(1) F_aero(2) F_aero(3)]';
%disp(F);

% Rotate the distance from CP to CG into the inertial frame
cp_vector = [0 0 dcp];
cg_vector = [0 0 dcg];

cp2cg_b     = cp_vector-cg_vector;
cp2cg_i     = cp2cg_b*R_bi; % distance from the cp to cg in inertial frame  

% Moment due to aerodynamic force of rocket body
M_fins_i = M_fins_b*R_bi;
M_aero   = cross(F_aero, cp2cg_i);

M=M_fins_i+M_aero;
%w_dot = I\(M-cross(w0,H));
w_dot = I'\(M'-cross(w0,H)); %this fixes size error

x_dot = [u; v; w; F(1)/m; F(2)/m; F(3)/m; q_dot; w_dot]; %13x1 column vector
%disp(x_dot);

vars = [u; v; w; q1; q2; q3; q4; wx; wy; wz];
A = jacobian(x_dot,vars); %13x13 matrix
%disp(A);
B = [zeros(11,4);jacobian(M_fins_i,[u1,u2,u3,u4])];
% C = [0 0 1 0 0 0 0 0 0 0 0 0 0;...
    %0 0 0 0 0 0 0 0 0 0 1 0 0;...
    %0 0 0 0 0 0 0 0 0 0 0 1 0;...
    %0 0 0 0 0 0 0 0 0 0 0 0 1]*x_dot;
%disp(C);
