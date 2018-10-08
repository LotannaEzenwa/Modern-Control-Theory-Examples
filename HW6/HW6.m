clear

A = .9;
B = 1;
C = 1;
phi = .1;

p = 50;
n = 50;

L = phi*eye(p);
x = zeros(1,p);
y = zeros(1,p);
u = zeros(1,p);
w = @(c,k)(sin(c*k));
E = [];


b = 0:p-1;
y_d = 1 - cos(pi*b/50);
%% PART A
clf

plot(1:p,y_d,'k--')
for j = 1:n
    hold on
    x = zeros(p,1);
    y = zeros(p,1);
    for k=1:p
        x(k+1) = A*x(k) + B*u(k) + w(.2,k);
        y(k) = x(k) + w(.1,k);
    end
    %Plot y(k)
    if mod(j,2) == 0
        plot(1:p,y,'r');
    end
    %Find Error
    e = y_d' - y;
    %Add to error matrix
    E(:,j) = e;
    %Calculate New Input
    for k = 1:p-1
        u(k) = u(k) + phi*E(k+1,j);
    end
end
legend('Desired Output','Iterations')
plot(1:p,y,'b')

clf
hold on
for ii=1:n
    plot(E(:,ii),'b');
end
close

%% PART B
% Choose Q, R, symmetric positive definite

Q = .2*rand(p,p);
R = 6000*rand(p,p);
Q = Q'*Q;
R = R'*R;

P = pmatrix(A,B,C,p);
u = zeros(p,1);
du = zeros(p,n);
e = zeros(p,n);
Y = [];
clf
for j = 1:n
    %Simulation
    x = zeros(p,1);
    y = zeros(p,1);
    for k=1:p
        x(k+1) = A*x(k) + B*u(k) + w(.2,k);
        y(k) = x(k) + w(.1,k);
    end
    %Calculate Error
    
    e(:,j) = y_d' - y;
    if j == 1
        
    end
    
    
    du(:,j) = (P'*Q*P + R)\P'*Q*e(:,j-1);
    u = u + du(:,j);
    Y(:,j) = y;
end

hold on
plot(y_d,'k--')
for ii=1:n
    plot(Y(:,ii),'b');
end


