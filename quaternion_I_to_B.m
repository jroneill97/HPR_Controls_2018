function B = quaternion_I_to_B(q,A)
% Rotation from Body fixed to Inertial frame
Q = dcm_from_q(q);
B = Q*A;