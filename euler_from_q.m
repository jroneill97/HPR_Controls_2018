function [yaw, pitch, roll] = euler_from_q(q)
[yaw,pitch,roll] = dcm_to_euler(dcm_from_q(q));

