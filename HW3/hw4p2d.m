clear
kc1 = 22;
zc1 = 42;
mc1 = 1;
kc2 = 10;
zc2 = 42;
mc2 = 1;
A_c = [0 0 0 0 1 0 0 0 
    0 0 0 0 0 1 0 0 
    0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 1
    -20-kc1 10 kc1 0 0 0 0 0 
    10 -10-kc1 0 kc2 0 0 0 0
    kc1/mc1 0 -kc1/mc1 0 0 0 -zc1/mc1 0 
    0 kc2/mc2 0 -kc2/mc2 0 0 0 -zc2/mc2
    ];
B_c = [0;0;0;0;0;0;0;0];
C = [1 0 0 0 0 0 0 0
    0 1 0 0 0 0 0 0];
D = 0;
system = ss(A_c,B_c,C,D);

dt = 0.1;
t = 0:dt:100;
u = zeros(size(t));
x0 = [-2;2;3;-2;1;4;-3;-4];
lsim(system,u,t,x0)
x0 = [-30;29;5;23;40;0;23;-2];
%lsim(system,u,t,x0)