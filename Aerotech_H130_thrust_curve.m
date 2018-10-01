% Aerotech H130W from TRA Cert Data

time_pts_H130= [0.011 .029 .04 .055 .086 .11 .147 .19 .226 .266 .304 .348 .4 .419 .432 .531 .601 1.184 1.216 1.351 1.402 1.633];   
thrust_H130=  [90.913 180.689 207.962 235.236 260.237 250.578 246.6 252.85 242.623 235.236 238.645 226.713 215.349 220.463 207.692 177.279 151.71 69.321 69.321 46.593 40.342 0];
new_time_pts_H130= 0:0.03125:1.633



figure
vq1 = interp1(time_pts_H130,thrust_H130,new_time_pts_H130);
plot(time_pts_H130,thrust_H130,'o',new_time_pts_H130,vq1,':.'); xlabel 'Time'; ylabel 'Thrust';
xlim([0 3]);
title('Aerotech H130W Thrust Curve (32 Hz)');