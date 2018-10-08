clear
A = [0 0 1 0
    0 0 0 1
    -20 10 0 0 
    10 -10 0 0];
B = [0; 0; 0; 1];
C = [1 0 0 0];
D = 0;
system = ss(A,B,C,D);
Q = .01*gallery('lehmer',4);
R = .1;
[K,S,E] = lqr(system,Q,R);
dt = 0.1;
t = 0:dt:100;
u = zeros(size(t));

fsys = ss(A-B*K,[0;0;0;1],[1 0 0 0
    K],D);
lsim(fsys,u,t,[-2;3;2;2])