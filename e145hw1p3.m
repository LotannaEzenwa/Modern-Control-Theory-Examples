clear
clf
syms t
A = [0 1
    -1 0];
expm(t*A);
dt = .01;
t = 0:dt:10;
u = ones(size(t));
A_c = [0 1
    -1  0];
B_c = [0 ; 1];
C = [0 1];
D = 0;
system = ss(A_c,B_c,C,D);
lsim(system,u,t,[0; 1]);
hold on
plot(t,cos(t) + sin(t) -.05,'r')
hold off
legend('linear_simulation', 'answers (a), (b) with .05 offset')