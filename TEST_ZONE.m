clc; clear all;

q = [0;0;1;0];

I = [1;2;3];
Q = quat2dcm(circshift(q,1)');
B = quatrotate(circshift(q,1)',I')'


Q = dcm_from_q(q);


B = Q*I


