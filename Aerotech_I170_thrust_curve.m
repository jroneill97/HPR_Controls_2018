clc;close all;clear variables
% Aerotech I170G from TRA Cert Data
%Created by Mark Hairfield 12-29-10
%I170G 54 151 10 0.227 0.528 AT
time_pts_I170= [.06 .16 0.25 .5 .75 1 1.5 2.18 2.25 2.3 2.35 2.4];
thrust_I170=  [89 169 184.6 202.4 207.3 202.4 177.9 137.9 139.7 131.2 22.2 0];
new_time_pts_I170= 0.06:.03125:2.4;

figure
vq1 = interp1(time_pts,thrust,new_time_pts);
x= vq1;
plot(time_pts,thrust,'o',new_time_pts,vq1,':.'); xlabel 'Time'; ylabel 'Thrust';
xlim([0 3]);
title('Aerotech I170 Thrust Curve (32 Hz)');
