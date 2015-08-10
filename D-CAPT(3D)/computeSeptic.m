function coeff = computeSeptic(x0,v0,a0,j0,xf,vf,af,jf, t0,tf)

B = [x0;v0;a0;j0;xf;vf;af;jf];
A = ...
[ 1, t0, t0^2,   t0^3,    t0^4,    t0^5,     t0^6,     t0^7
    0,  1, 2*t0, 3*t0^2,  4*t0^3,  5*t0^4,   6*t0^5,   7*t0^6
    0,  0,    2,   6*t0, 12*t0^2, 20*t0^3,  30*t0^4,  42*t0^5
    0,  0,    0,      6,   24*t0, 60*t0^2, 120*t0^3, 210*t0^4
    1, tf, tf^2,   tf^3,    tf^4,    tf^5,     tf^6,     tf^7
    0,  1, 2*tf, 3*tf^2,  4*tf^3,  5*tf^4,   6*tf^5,   7*tf^6
    0,  0,    2,   6*tf, 12*tf^2, 20*tf^3,  30*tf^4,  42*tf^5
    0,  0,    0,      6,   24*tf, 60*tf^2, 120*tf^3, 210*tf^4];
coeff = A\B;
end