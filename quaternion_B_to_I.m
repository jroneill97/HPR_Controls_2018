function B = quaternion_B_to_I(q,A)
% Rotation from Body fixed to Inertial frame
Q = dcm_from_q(q);
B = Q'*A;