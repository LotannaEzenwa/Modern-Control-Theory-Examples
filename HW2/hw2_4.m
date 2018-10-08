clear
A_c = [0 1
    -10  0];
B_c = [0 ; 1];
C = [1 0; 1 0];
D = [0; 0];
system = ss(A_c,B_c,C,D);
dt = .1;
x = c2d(system,dt);
A = x.A;
B = x.B;
syms f1 f2
K = [f1 f2]
A_ = A + B*K;
syms s
eq1 = det(s*[1 0;0 1] - A_) == 0


%Part C
A_bar = A + B*[-90 -14];


t = [1:100;1:100];
x = zeros(size(t));
x_u = zeros(size(t));
y = zeros(1,100);
y_u = zeros(1,100);
x(1,1) = 1;
x(2,1) = 0;
x_u(1,1) = 1;
x_u(2,1) = 0;
for n = 2:100
    x(:,n) = A_bar*[x(1,n-1); x(2,n-1)];
    x_u(:,n) = A*[x_u(1,n-1); x_u(2,n-1)];
    y(n) = [1 0]*x(:,n);
    y_u(n) = [1 0]*x_u(:,n);
    
end

hold on
plot(t(1,:)*.1,y)
plot(t(1,:)*.1,y_u)
xlabel('time (s)')
title('Deadbeat System Response')
legend('closed loop','open loop')

%Part D
hold off
clf
hold off

plot(t(1,:)*.1,x)
title('Closed Loop Time Histories')
xlabel('time (s)')
legend('position','velocity')
