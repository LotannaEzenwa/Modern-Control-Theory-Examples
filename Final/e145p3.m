clear
clf
format shortE
clear
M = eye(8)*100;
K = [27071.1 0 0 0 -10000 0 -3535.5 -3535.5
    0 17071.1 0 -10000 0 0 -3535.5 -3535.5
    0 0 27071.1 0 -3535.5 -3535.5 -10000 0
    0 -10000 0 17071.1 3535.5 -3535.5 -10000 0 
    -10000 0 -3535.5 3535.5 27071.1 0 0 0
    0 0 3535.5 -3535.5 0 17071.1 0 -10000
    -3535.5 -3535.5 -10000 0 0 0 27071.1 0
    -3535.5 -3535.5 0 0 0 -10000 0 17071.1];

%% Model
A_c = [zeros(8) eye(8)
    -inv(M)*K zeros(8)];

dt = 0.02
ac = A_c;
b_f = zeros(8,4);
b_f(1,1) = 1;
b_f(2,2) = 1;
b_f(7,3) = 1;
b_f(8,4) = 1;
b_v = inv(M)*b_f;
B_c = [zeros(8,4)
    b_v];

bc = B_c;
C_0= zeros(4,16);
C_0(1,1) = 1;
C_0(2,2) = 1;
C_0(3,5) = 1;
C_0(4,6) = 1;
c = C_0;
d = 0;
system = ss(A_c,B_c,C_0,0);
disc0 = c2d(system, dt);
A0 = disc0.A;
B0 = disc0.B;
C0 = disc0.C;
D0 = disc0.D;
dimpulse(A0,B0,C0,D0,[1 2 3 4], 500)

a = A0;
b = B0;
p = pole(disc0);
K = place(A0,B0, .99*p);
e = eig(A0-B0*K);


subplot(1,3,1)
initialState = ones(16,1);
C_t = eye(16);
u = zeros(4,500);
dlsim(A0,B0,C_t,0,u,initialState');
subplot(1,3,2)
dlsim(A0-B0*K,B0,C_t,0,u,initialState');
subplot(1,3,3)
dlsim(A0-B0*K,B0,K,0,u,initialState');
