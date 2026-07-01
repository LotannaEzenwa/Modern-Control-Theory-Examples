%% Nyquist Stability Criterion
% *Closed-loop stability from encirclements of $-1$.*
%
% Ogata, _Modern Control Engineering_, Ch. 7.
%
% In this tutorial you will:
%
% * apply $Z=N+P$ to read closed-loop stability off a Nyquist plot,
% * work a stable-open-loop and an unstable-open-loop ($P>0$) example, and
% * watch a stable design lose stability as the gain crosses $K_{cr}$.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% For a unity-feedback loop with open-loop transfer function $G(s)H(s)$,
% the Nyquist criterion relates the number of closed-loop right-half-plane
% (RHP) poles $Z$ to the number of open-loop RHP poles $P$ and the number
% of clockwise encirclements $N$ of the point $-1+j0$ made by the Nyquist
% plot of $G(j\omega)H(j\omega)$ as $\omega$ traverses the Nyquist
% contour:
%
% $Z = N + P$
%
% The closed-loop system is stable iff $Z=0$, i.e. $N=-P$ (the locus must
% encircle $-1$ counterclockwise exactly $P$ times).

%% Worked Example: Stable Open Loop, Determine Closed-Loop Stability
% $G(s) = \frac{K}{(s+1)(s+2)(s+3)}$. The open-loop system has $P=0$ RHP
% poles (all open-loop poles are in the LHP), so for closed-loop
% stability we need $N=0$ -- the Nyquist plot must NOT encircle $-1$.
K = 60;
G = tf(K,conv(conv([1 1],[1 2]),[1 3]));
fprintf('Open-loop poles:\n'); disp(pole(G))

figure
nyquist(G)
title('Nyquist Plot: $G(s)=\frac{60}{(s+1)(s+2)(s+3)}$','Interpreter','latex','FontSize',20)
grid on

T = feedback(G,1);
cl_poles = pole(T);
fprintf('Closed-loop poles at K=%d:\n', K); disp(cl_poles)
n_rhp = sum(real(cl_poles)>0);
fprintf('Number of closed-loop RHP poles (Z) = %d\n', n_rhp)

%% Critical Gain from the Nyquist/Routh Boundary
% The characteristic equation is $(s+1)(s+2)(s+3)+K=0$, i.e.
% $s^3+6s^2+11s+(6+K)=0$. By Routh's array (see
% |Transient and Steady-State/RouthStability.m|), stability requires
% $0<K<60$ -- so $K=60$ above is exactly the marginal case, and the
% Nyquist plot should pass through $-1+j0$.
Kcr = 60;
fprintf('Predicted critical gain Kcr = %d (locus passes through -1)\n', Kcr)

%% Worked Example: Unstable Open-Loop Plant (P > 0)
% $G(s) = \frac{K}{s(s-1)}$ has one open-loop RHP pole ($P=1$). For
% closed-loop stability we now need $N=-1$, i.e. exactly one
% *counterclockwise* encirclement of $-1$.
G3 = tf(1,[1 -1 0]);
fprintf('Open-loop poles of G3:\n'); disp(pole(G3))

figure
nyquist(G3)
title('Nyquist Plot: $G(s)=\frac{1}{s(s-1)}$ (Unstable Open Loop, P=1)','Interpreter','latex','FontSize',20)
grid on

T3 = feedback(G3,1);
fprintf('Closed-loop poles of G3/(1+G3):\n'); disp(pole(T3))
fprintf('Closed-loop stable: %d\n', all(real(pole(T3))<0))

%% Before vs. After: Crossing the Critical Gain
% Back to the first plant. Below Kcr the Nyquist plot stays to the right
% of -1 (no encirclement, closed-loop stable); above Kcr it encircles -1
% (Z = 2, unstable). This is the same stability boundary the root locus
% crosses at K = Kcr.
den3 = conv(conv([1 1],[1 2]),[1 3]);
figure
subplot(1,2,1)
nyquist(tf(30,den3))
title('Before: $K=30<K_{cr}$ (stable)','Interpreter','latex','FontSize',13)
grid on
subplot(1,2,2)
nyquist(tf(90,den3))
title('After: $K=90>K_{cr}$ (encircles $-1$)','Interpreter','latex','FontSize',13)
grid on
fprintf('K=30: closed-loop RHP poles = %d;  K=90: closed-loop RHP poles = %d\n', ...
    sum(real(pole(feedback(tf(30,den3),1)))>0), ...
    sum(real(pole(feedback(tf(90,den3),1)))>0))

%% Try it yourself
% * Sweep K between 30 and 90 and find (by counting encirclements of -1)
%   the gain where the closed loop first goes unstable -- it is Kcr=60.
% * For the unstable-open-loop plant, check that closing the loop is
%   actually stable by inspecting |pole(feedback(G3,1))|.
