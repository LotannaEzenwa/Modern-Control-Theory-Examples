clear
B = [1 0 0 0 0 0 0 0]
M = eye(8)*100;
K = [27071.1 0 0 0 -10000 0 -3535.5 -3535.5
    0 17071.1 0 -10000 0 0 -3535.5 -3535.5
    0 0 27071.1 0 -3535.5 -3535.5 -10000 0
    0 -10000 0 17071.1 3535.5 -3535.5 -10000 0 
    -10000 0 -3535.5 3535.5 27071.1 0 0 0
    0 0 3535.5 -3535.5 0 17071.1 0 -10000
    -3535.5 -3535.5 -10000 0 0 0 27071.1 0
    -3535.5 -3535.5 0 0 0 -10000 0 17071.1];

ac1 = inv([zeros(8) M 
    M zeros(8)]);
ac2 = [-K zeros(8)
    zeros(8) M];


A_c = ac1*ac2;
system = ss(A_c,zeros(1,16)',zeros(1,16),0);
disc = c2d(system,0.02);
A = disc.A

B = zeros(1,16)';
B(1) = 1;
B(2) = 1;

system = ss(A_c,B, zeros(1,16),0);
disc = c2d(system,0.02);
B = disc.B;
c0 = ctrb(A,B);
[U1,S1,V1] = svd(full(c0));
semilogy(diag(S1),'r.');
v = diag(S1);
v(v<exp(10e-10)) = [];
v
k = max(v)/min(v)
%% PartC

hold on
%Test Node 2
B = zeros(1,16)';
B(1) = 1;
B(2) = 1;
B(3) = 1;
B(4) = 1;

system = ss(A_c,B, zeros(1,16),0);
disc = c2d(system,0.02);
B = disc.B;
c0 = ctrb(A,B);
[U1,S1,V1] = svd(full(c0));
semilogy(diag(S1),'r.');
v = diag(S1);
v(v<exp(10e-10)) = [];
v
k = max(diag(v))/min(diag(v))
% k = 6.2057

%Test Node 3
B = zeros(1,16)';
B(1) = 1;
B(2) = 1;
B(5) = 1;
B(6) = 1;

system = ss(A_c,B, zeros(1,16),0);
disc = c2d(system,0.02);
B = disc.B;
c0 = ctrb(A,B);
[U1,S1,V1] = svd(full(c0));

semilogy(diag(S1),'r.');
v = diag(S1);
v(v<exp(10e-10)) = [];
v
k = max(diag(v))/min(diag(v))


% k = 8.0148e+16

%Test Node 4
B = zeros(1,16)';
B(1) = 1;
B(2) = 1;
B(7) = 1;
B(8) = 1;

system = ss(A_c,B, zeros(1,16),0);
disc = c2d(system,0.02);
B = disc.B;
c0 = ctrb(A,B);
[U1,S1,V1] = svd(full(c0));
semilogy(diag(S1),'r.');
v = diag(S1);
v(v<exp(10e-10)) = [];
v
k = max(v)/min(v)
% 1.2071e+17
