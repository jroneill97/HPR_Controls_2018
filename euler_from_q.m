function [yaw, pitch, roll] = euler_from_q(q)
Q = dcm_from_q(q);
[yaw,pitch,roll] = dcm_to_euler(Q);
yaw   = deg2rad(yaw);
pitch = deg2rad(pitch);
roll  = deg2rad(roll);

