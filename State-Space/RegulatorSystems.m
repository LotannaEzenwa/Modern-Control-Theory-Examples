%% Regulator Systems: Combined State Feedback and Observer
% Ogata, Modern Control Engineering, Ch. 10: Observed-State Feedback
% Control Systems
%
% When the full state is not measurable, the *separation principle*
% justifies designing the state-feedback gain $K$ (from |PolePlacement.m|)
% and the observer gain $G$ (from |StateObservers.m|) *independently*:
% the closed-loop system built from $u=-K\hat{x}$ with $\hat{x}$ supplied
% by the observer has characteristic polynomial equal to the *product*
% of the controller's and the observer's individually-designed
% polynomials. The combined controller-observer is called an
% *observer-based regulator* (or, with a reference input, a *compensator*).

%% Plant
A = [0 1 0; 0 0 1; 0 -2 -3];
B = [0;0;1];
C = [1 0 0];
D = 0;
n = size(A,1);

%% Step 1: Design State-Feedback Gain (Controller Poles)
controller_poles = [-2+2j,-2-2j,-6];
K = acker(A,B,controller_poles);
fprintf('Controller gain K = '); disp(K)

%% Step 2: Design Observer Gain (Observer Poles, Faster)
observer_poles = [-10,-11,-12];
G = acker(A',C',observer_poles)';
fprintf('Observer gain G = '); disp(G)

%% Step 3: Augmented Closed-Loop System
% State the combined dynamics in terms of $[x;\ e]$ where $e=x-\hat{x}$:
%
% $$\dot{x} = Ax - BK\hat{x} = (A-BK)x + BKe$$
%
% $$\dot{e} = (A-GC)e$$
%
% This block-triangular structure is the separation principle made
% explicit: the $2n\times 2n$ system matrix
%
% $$\begin{bmatrix}A-BK & BK\\0 & A-GC\end{bmatrix}$$
%
% has eigenvalues equal to the union of $\mathrm{eig}(A-BK)$ and
% $\mathrm{eig}(A-GC)$.
A_aug = [A-B*K, B*K; zeros(n), A-G*C];
fprintf('\nCombined system eigenvalues:\n')
disp(eig(A_aug)')
fprintf('Controller poles ∪ observer poles:\n')
disp([controller_poles observer_poles])

%% Simulating the Regulator
% Initial-condition (regulator) response: the plant starts away from
% equilibrium, the observer starts at zero (so $e(0)=x(0)$), and the
% combined system is driven to zero by output feedback through the
% estimated state.
x0 = [1;0;0];
e0 = x0;   % observer starts at zero estimate
z0 = [x0;e0];
C_aug = [eye(n) zeros(n); eye(n) -eye(n)];   % outputs: x and x_hat=x-e
sys_aug = ss(A_aug, zeros(2*n,1), C_aug, zeros(2*n,1));
t = 0:0.01:5;
[z,~] = initial(sys_aug,z0,t);
x_resp = z(:,1:n);
xhat_resp = z(:,n+1:end);

figure
plot(t,x_resp(:,1),'b',t,xhat_resp(:,1),'r--')
legend('$x_1$ (true)','$\hat{x}_1$ (estimated)','Interpreter','latex','FontSize',14)
title('Observer-Based Regulator: Initial-Condition Response','Interpreter','latex','FontSize',20)
ylabel('$x_1(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Transfer Function of the Observer-Based Compensator
% The dynamic output-feedback compensator that implements $u=-K\hat{x}$
% internally (eliminating $\hat{x}$ as an explicit state visible to the
% plant) has its own transfer function $G_c(s)$ from $-y$ to $u$:
%
% $$G_c(s) = K(sI-A+BK+GC)^{-1}G$$
A_c = A - B*K - G*C;
B_c = G;
C_c = -K;
D_c = 0;
Gc_compensator = tf(ss(A_c,B_c,C_c,D_c));
fprintf('\nObserver-based compensator transfer function:\n')
Gc_compensator
