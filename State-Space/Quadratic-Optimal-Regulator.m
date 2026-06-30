%% Quadratic Optimal Regulator (LQR)
% Ogata, Modern Control Engineering, Ch. 10: Quadratic Optimal Control
%
% Rather than placing poles at hand-chosen locations, the *linear
% quadratic regulator* (LQR) chooses the feedback gain $K$ in $u=-Kx$ to
% minimize a performance index trading off state deviation against
% control effort:
%
% $$J = \int_0^\infty \left(x^TQx + u^TRu\right)dt$$
%
% where $Q\ge0$ weights state error and $R>0$ weights control effort. The
% optimal gain is $K=R^{-1}B^TP$, where $P=P^T\ge0$ solves the algebraic
% Riccati equation (ARE):
%
% $$A^TP+PA-PBR^{-1}B^TP+Q=0$$

%% Plant
A = [0 1; -1 -1];
B = [0; 1];
C = [1 0];
D = 0;

%% Solving the Riccati Equation
% `care` solves the continuous-time ARE directly; `lqr` wraps this and
% additionally returns the optimal gain $K$ and closed-loop eigenvalues.
Q = diag([10 1]);
R = 1;
[K,P,clp] = lqr(A,B,Q,R);
fprintf('LQR gain K = '); disp(K)
fprintf('Riccati solution P = \n'); disp(P)
fprintf('Closed-loop poles = '); disp(clp')

%%
% Cross-check via `care` directly: $A^TP+PA-PBR^{-1}B^TP+Q=0$.
[P_care,~,K_care] = care(A,B,Q,R);
fprintf('\ncare() gain (= R^{-1}B^TP, sign convention matches lqr K): ')
disp(K_care)
residual = A'*P_care + P_care*A - P_care*B*(R\B')*P_care + Q;
fprintf('||Riccati residual|| = %.2e (should be ~0)\n', norm(residual))

%% Effect of Q and R on the Tradeoff
% Increasing $Q$ (penalize state error more) drives faster, more
% aggressive poles; increasing $R$ (penalize control effort more) yields
% gentler, slower poles and smaller gains. This is the central LQR design
% knob, replacing the trial-and-error of direct pole placement.
[K_aggressive,~,~] = lqr(A,B,diag([100 1]),1);
[K_gentle,~,~]      = lqr(A,B,diag([1 1]),10);
fprintf('\nAggressive (Q11=100,R=1):  K = '); disp(K_aggressive)
fprintf('Gentle (Q11=1,R=10):       K = '); disp(K_gentle)

sys_aggr = ss(A-B*K_aggressive,B,C,D);
sys_gentle = ss(A-B*K_gentle,B,C,D);
figure
hold on
step(sys_aggr)
step(sys_gentle)
hold off
legend('Aggressive ($Q_{11}=100,R=1$)','Gentle ($Q_{11}=1,R=10$)', ...
    'Interpreter','latex','FontSize',14)
title('LQR: Effect of $Q/R$ Weighting on Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Guaranteed Stability and Robustness Margins
% A classical result: full-state-feedback LQR (with $R$ scalar) always
% achieves at least 60-degree phase margin and infinite gain margin (gain
% reduction tolerance) at the plant input -- a robustness guarantee that
% generic pole placement does not provide.
sys_ol_lqr = ss(A,B,K,0);
[~,PM_lqr] = margin(sys_ol_lqr);
fprintf('\nLQR loop-gain phase margin at plant input = %.2f deg (>=60 guaranteed)\n', PM_lqr)
