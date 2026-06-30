%% Pole Placement via State Feedback
% Ogata, Modern Control Engineering, Ch. 10: Pole Placement
%
% If $(A,B)$ is completely controllable, full-state feedback
% $u=-Kx$ (plus a reference input $r$, $u=-Kx+r$) places the closed-loop
% poles -- the eigenvalues of $A-BK$ -- at *any* desired location in the
% complex plane (subject to complex poles occurring in conjugate pairs).
% The closed-loop characteristic equation is
%
% $$|sI-(A-BK)| = 0$$

%% Plant
% $G(s)=\frac{1}{s(s+1)(s+2)}$ in controllable canonical form.
A = [0 1 0; 0 0 1; 0 -2 -3];
B = [0;0;1];
C = [1 0 0];
D = 0;
fprintf('Open-loop poles: '); disp(eig(A)')
fprintf('Controllability rank = %d (n=%d)\n', rank(ctrb(A,B)), size(A,1))

%% Ackermann's Formula
% For SISO systems, the feedback gain placing the poles at the roots of a
% desired characteristic polynomial $\alpha_c(s)=s^n+\alpha_1s^{n-1}+
% \cdots+\alpha_n$ is given directly by
%
% $$K = \begin{bmatrix}0&0&\cdots&1\end{bmatrix}\mathcal{C}^{-1}\alpha_c(A)$$
%
% where $\mathcal{C}$ is the controllability matrix and $\alpha_c(A)$ is
% the matrix polynomial (Cayley-Hamilton-style substitution of $A$ for
% $s$).
desired_poles = [-2+2j, -2-2j, -10];
K_acker = acker(A,B,desired_poles);
fprintf('\nAckermann gain K = '); disp(K_acker)
fprintf('Resulting closed-loop poles: '); disp(eig(A-B*K_acker)')

%%
% Manual Ackermann computation (for verification): build
% $\alpha_c(s)=(s-p_1)(s-p_2)(s-p_3)$, then evaluate $\alpha_c(A)$.
alpha_c = poly(desired_poles);   % desired char. polynomial coefficients
alpha_c_A = alpha_c(1)*A^3 + alpha_c(2)*A^2 + alpha_c(3)*A + alpha_c(4)*eye(3);
Cc = ctrb(A,B);
K_manual = [0 0 1]*(Cc\alpha_c_A);
fprintf('Manual Ackermann K = '); disp(K_manual)

%% place() for Robust Multi-Pole Placement
% `place` uses a numerically robust algorithm (Kautsky-Nichols-Van Dooren)
% and additionally supports multi-input systems; it requires distinct
% desired poles.
K_place = place(A,B,desired_poles);
fprintf('\nplace() gain K = '); disp(K_place)

%% Closed-Loop Step Response with Reference Tracking
% With $u=-Kx+N_rr$, choose $N_r$ so the DC gain from $r$ to $y$ is 1
% (zero steady-state error to a step reference), assuming $D=0$:
%
% $$N_r = \frac{-1}{C(A-BK)^{-1}B}$$
Nr = -1/(C*((A-B*K_acker)\B));
sys_cl = ss(A-B*K_acker, B*Nr, C, D);
figure
step(sys_cl)
title('Pole-Placement Closed-Loop Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Effect of Desired Pole Location on Response Speed and Control Effort
% Faster (more negative) poles give a quicker response but typically
% larger gains $K$ and larger initial control effort.
poles_slow = [-1+1j,-1-1j,-5];
poles_fast = [-4+4j,-4-4j,-20];
K_slow = acker(A,B,poles_slow);
K_fast = acker(A,B,poles_fast);
fprintf('\n||K_slow|| = %.2f, ||K_fast|| = %.2f\n', norm(K_slow), norm(K_fast))

Nr_slow = -1/(C*((A-B*K_slow)\B));
Nr_fast = -1/(C*((A-B*K_fast)\B));
sys_slow = ss(A-B*K_slow, B*Nr_slow, C, D);
sys_fast = ss(A-B*K_fast, B*Nr_fast, C, D);
figure
hold on
step(sys_slow)
step(sys_fast)
hold off
legend('Slower poles','Faster poles','Interpreter','latex','FontSize',14)
title('Pole Location vs. Response Speed','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)
