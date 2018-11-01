% Linearize the equations of motion (need to do this every time the rocket
% parameters change!!!!)
clc; clear all;
load rocket
global aeroConstant g Fg_i
aeroConstant  = 0.5*(1.225)*rocket.area;
g    = 9.8;
Fg_i = [0; 0; -g*rocket.m];


syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 T Fx Fy fin1 fin2 fin3 fin4

states   = [x1 ;x2; x3; x4 ;x5 ;x6; x7 ;x8 ;x9; x10; x11; x12; x13];
x   = states;
thrust_b = [0;0;T];
dFins    = [fin1;fin2;fin3;fin4];

xDot     = EquationsOfMotion(x,rocket,thrust_b,Fx,Fy,dFins);

A        = jacobian(xDot,states); % <<<< A is the linearized matrix

f =  matlabFunction(A,'File','A_Matrix','Optimize',false);

clear all;
load handel.mat;
sound(y);