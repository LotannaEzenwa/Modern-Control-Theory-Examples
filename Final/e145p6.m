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

%% Part A
p = .9*eig(A0);
L = place(A0',C0',p);
%% Test Seed

initialState = ones(16,1);

randn('seed',1);
u = randn(500,4);
u_o = zeros(500,4);
clf
figure(1)
dlsim(A0,B0,C0,D0,u,initialState);
figure(2)
xhat = zeros(500,16);
x_s = zeros(500,16);
y_s = zeros(500,4);
yhat = zeros(500,4);
x_s(1,:) = ones(1,16);

for i = 1:500
    y_s(i,:) = C0*x_s(i,:)';
    yhat(i,:) = C0*xhat(i,:)';
    x_s(i+1,:) = A0*x_s(i,:)' + B0*u(i,:)';
    xhat(i+1,:) = (A0 - L'*C0)*xhat(i,:)' + B0*u(i,:)' + L'*y_s(i,:)';
end
plot(yhat-y_s)
