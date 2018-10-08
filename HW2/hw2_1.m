clear
A_c = [0 1
    -10  0];
B_c = [0 ; 1];
C = [1 0];
D = 0;
system = ss(A_c,B_c,C,D);
x = c2d(system,.1)
A = x.A;
C = A_c
dt = .1
c = [1 0
    0 1]+ dt*A_c + 1/2*(dt*A_c)^2 + 1/6*(dt*A_c)^3 + 1/24*(dt*A_c)^4 + 1/120*(dt*A_c)^5
