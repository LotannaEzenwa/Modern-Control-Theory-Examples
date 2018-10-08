clear
format shortE
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

b_f = zeros(8,4);
b_f(1,1) = 1;
b_f(2,2) = 1;
b_f(7,3) = 1;
b_f(8,4) = 1;
b_v = inv(M)*b_f;
B_c = [zeros(8,4)
    b_v];


C_0= zeros(4,16);
C_0(1,1) = 1;
C_0(2,2) = 1;
C_0(3,5) = 1;
C_0(4,6) = 1;

system = ss(A_c,B_c,C_0,0);
disc0 = c2d(system, 0.02);
A0 = disc0.A;
B0 = disc0.B;
C0 = disc0.C;
D0 = disc0.D;

q = 40;

Aq = A0^q;

T = []
for i = 1:q
    T_i = A0^(q-i)*B0;
    T = [T T_i];
end

pT = -pinv(T);
F = pT(1:4,:)*Aq;

subplot(1,3,1)
initialState = ones(16,1);
C_t = eye(16);
u = zeros(4,500);
dlsim(A0,B0,C_t,0,u,initialState')
subplot(1,3,2)
dlsim(A0 + B0*F,B0,C_t,0,u,initialState');
subplot(1,3,3)
dlsim(A0 + B0*F,B0, F,0,u,initialState');


%% Part C
q = 120;

Aq = A0^q;

T = []
for i = 1:q
    T_i = A0^(q-i)*B0;
    T = [T T_i];
end

pT = -pinv(T);
F = pT(1:4,:)*Aq;
clf
subplot(1,3,1)
initialState = ones(16,1);
C_t = eye(16);
u = zeros(4,500);
dlsim(A0,B0,C_t,0,u,initialState')
subplot(1,3,2)
dlsim(A0 + B0*F,B0,C_t,0,u,initialState');
subplot(1,3,3)
dlsim(A0 + B0*F,B0, F,0,u,initialState');