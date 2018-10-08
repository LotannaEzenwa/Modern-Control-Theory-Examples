clear
A_c = [0 1
    -10  0];
B_c = [0 ; 1];
C = [1 0];
D = [0];
system = ss(A_c,B_c,C,D);
dt = .1;
x = c2d(system,dt);