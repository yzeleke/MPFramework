function [a5;a4;a3;a2;a1;a0] = ppp(x0,v0,a0,xf,vf,af,tf)

B = [x0;v0;a0;xf;vf;af];
A = [0,       0,       0,      0,     0,     0;...
     0,       0,       0,      0,     1,     0;...
     0,       0,       0,      2,     0,     0;...
     tf^5,    tf^4,    tf^3,   tf^2,  tf,    1;...
     5*tf^4,  4*tf^3,  3*tf^2, 2*tf,  1,     0;...
     20*tf^3, 12^tf^2, 6*tf,   2,     0,     0];
[a5;a4;a3;a2;a1;a0] = A\B;



