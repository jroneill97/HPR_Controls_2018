function q = quat_from_ypr(psi,theta,phi)
Q = [-sin(phi)*cos(theta)*sin(psi)+cos(phi)*cos(psi),cos(phi)*cos(theta)*sin(psi)+sin(phi)*cos(psi),sin(theta)*sin(psi);
      -sin(phi)*cos(theta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi),sin(theta)*cos(psi);
       sin(phi)*sin(theta),-cos(phi)*sin(theta),cos(theta)];
q = q_from_dcm(Q);