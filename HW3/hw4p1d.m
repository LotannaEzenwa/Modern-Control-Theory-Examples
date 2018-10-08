clear
kc = 13;
zc = 3;
mc = .6;

A_c = [0 0 0 1 0 0
    0 0 0 0 1 0 
    0 0 0 0 0 1
    -20 10 0 0 0 0
    10 -10-kc kc 0 0 0
    0 kc/mc -kc/mc 0 0 -zc/mc
    ];
B_c = [0;0;0;0;0;0];
C = [0 1 0 0 0 0];
D = 0;
system = ss(A_c,B_c,C,D);

dt = 0.1;
t = 0:dt:10;
u = ones(size(t));
x0 = [-2;2;3;-2;1;4];
lsim(system,u,t,x0)
x0 = [-30;29;5;23;40;0];
lsim(system,u,t,x0)