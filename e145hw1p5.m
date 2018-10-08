clear
A_c = [0 1
    -1  0];
B_c = [0 ; 1];
C = [1 0];
D = 0;
system = ss(A_c,B_c,C,D);
dsym = c2d(system,0.1);
dt = 0.1;
t = 0:dt:1;
u = sin(t);
dlsim(dsym.A, dsym.B, dsym.C, dsym.D, u)
