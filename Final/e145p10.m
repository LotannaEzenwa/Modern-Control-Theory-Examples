%% E145 Final, Problem 10
% NOTE: the original assignment text for this problem is not present in
% this repository (this file was found duplicated from an unrelated
% script). The following is an inferred, plausible continuation problem
% in the style of the other Final/e145p*.m files -- discrete-time
% controllability and pole-placement design -- not the original
% assignment text.
%
% Plant: a discretized double-integrator (e.g. a position/velocity pair
% sampled every $T=0.2$s), tested for controllability and then driven to
% the origin from a nonzero initial condition via state feedback placed
% at specified discrete-time pole locations.

%% Discretize the Plant
A_c = [0 1; -1 0];
B_c = [0;1];
C = [1 0];
D = 0;
T = 0.2;

sys_c = ss(A_c,B_c,C,D);
sys_d = c2d(sys_c,T);
A = sys_d.A; B = sys_d.B; C_d = sys_d.C; D_d = sys_d.D;

fprintf('Discrete plant A = \n'); disp(A)
fprintf('Discrete plant B = \n'); disp(B)

%% Part (a): Controllability
% Form the controllability matrix $\mathcal{C}=[B\ AB]$ and check rank.
Co = ctrb(A,B);
fprintf('\nrank(Co) = %d (n = %d) -> controllable: %d\n', ...
    rank(Co), size(A,1), rank(Co)==size(A,1))

%% Part (b): Deadbeat-ish Pole Placement
% Place the closed-loop poles of $u=-Kx$ at $z=0.2,0.3$ (well inside the
% unit circle) using Ackermann's formula.
desired_poles = [0.2 0.3];
K = acker(A,B,desired_poles);
fprintf('\nState-feedback gain K = '); disp(K)
fprintf('Closed-loop poles = '); disp(eig(A-B*K)')

%% Part (c): Regulation from a Nonzero Initial Condition
% Simulate $x_{k+1}=(A-BK)x_k$ from $x_0=[1;-1]$ and confirm the state
% decays to the origin within a handful of samples (consistent with the
% fast placed poles).
N = 15;
x0 = [1;-1];
x_hist = zeros(2,N+1);
x_hist(:,1) = x0;
for k = 1:N
    x_hist(:,k+1) = (A-B*K)*x_hist(:,k);
end

figure
stairs(0:N,x_hist')
legend('$x_1$ (position)','$x_2$ (velocity)','Interpreter','latex','FontSize',14)
title('Problem 10: Deadbeat-Style Regulation','Interpreter','latex','FontSize',20)
ylabel('$x_k$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$k$','Interpreter','latex','FontSize',20)

fprintf('\nFinal state norm after %d steps: %.2e\n', N, norm(x_hist(:,end)))
