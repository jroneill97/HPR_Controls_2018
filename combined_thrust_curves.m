clc; close all; clear variables

time_pts_I170= [.06 .16 0.25 .5 .75 1 1.5 2.18 2.25 2.3 2.35 2.4];
thrust_I170=  [89 169 184.6 202.4 207.3 202.4 177.9 137.9 139.7 131.2 22.2 0];

time_pts_H130= [0.011 .029 .04 .055 .086 .11 .147 .19 .226 .266 .304 .348 .4 .419 .432 .531 .601 1.184 1.216 1.351 1.402 1.633];   
thrust_H130=  [90.913 180.689 207.962 235.236 260.237 250.578 246.6 252.85 242.623 235.236 238.645 226.713 215.349 220.463 207.692 177.279 151.71 69.321 69.321 46.593 40.342 0];


new_time_pts_H130= [0.011:.03125:1.633];
new_time_pts_I170= [0.06:.03125:2.4];
new_time_pts= [0:.03125:3];

vq1_I170 = [interp1(time_pts_I170,thrust_I170,new_time_pts)]
vq1_H130 = [interp1(time_pts_H130,thrust_H130,new_time_pts)];

total_thrust= vq1_I170 + vq1_H130

plot(new_time_pts,total_thrust); xlabel 'Time'; ylabel 'Thrust';
xlim([0 3]);
title('Total Thrust Curve (32 Hz)');


