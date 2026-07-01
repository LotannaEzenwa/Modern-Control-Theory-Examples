%% Mathematical Models -- Worked Problems
% *Practice: ODE-to-TF, parameter identification, and block reduction.*
%
% Ogata, _Modern Control Engineering_, Ch. 2--4 (end-of-chapter style).
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.

%% Problem 1: ODE to Transfer Function
% Find the transfer function $G(s)=Y(s)/U(s)$ for the system
%
% $\dddot{y} + 6\ddot{y} + 11\dot{y} + 6y = 6u$
%
% Taking the Laplace transform with zero initial conditions:
%
% $(s^3+6s^2+11s+6)Y(s) = 6U(s) \quad\Rightarrow\quad
%   G(s) = \frac{6}{s^3+6s^2+11s+6}$
G1 = tf(6,[1 6 11 6]);
poles_1 = pole(G1)

%%
% The poles at $s=-1,-2,-3$ are all in the left half-plane, so the
% system is stable; the step response should settle without sustained
% oscillation.
figure
step(G1)
title('Problem 1: Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Problem 2: Spring-Mass-Damper Parameter Identification
% A mass-spring-damper system ($m\ddot{x}+b\dot{x}+kx=u$) with
% $m=2\,\mathrm{kg}$ is observed to have natural frequency
% $\omega_n=3\,\mathrm{rad/s}$ and damping ratio $\zeta=0.5$. Recall
% the standard second-order form
%
% $\frac{1/m}{s^2 + (b/m)s + (k/m)} =
%   \frac{1/m}{s^2+2\zeta\omega_n s + \omega_n^2}$
%
% so $k = m\omega_n^2$ and $b = 2\zeta\omega_n m$.
m = 2; wn = 3; zeta = 0.5;
k = m*wn^2;
b = 2*zeta*wn*m;
fprintf('k = %.2f N/m, b = %.2f N.s/m\n', k, b)

G2 = tf(1/m,[1 b/m k/m]);
figure
step(G2)
title('Problem 2: Identified System Step Response','Interpreter','latex','FontSize',20)
ylabel('$x(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Problem 3: Block-Diagram Reduction
% Reduce the cascaded, unity-feedback system with inner loop
% $G_1(s)=\frac{1}{s+2}$, forward gain $G_2(s)=\frac{5}{s}$, and outer
% unity feedback, to a single closed-loop transfer function, then
% verify its steady-state step value via the final value theorem.
G1 = tf(1,[1 2]);
G2 = tf(5,[1 0]);
fwd = series(G1,G2);          % forward path only (before closing the loop)
T = feedback(fwd,1)            % closed-loop (after)

ess = 1 - dcgain(T);   % steady-state error for unit step (type-1 system, ess should be 0)
fprintf('Steady-state error to unit step = %.4f\n', ess)

%% Problem 3 -- Before vs. After Closing the Loop
% The open forward path $G_1G_2$ has an integrator, so its step ramps off
% without bound (before); closing the unity-feedback loop turns it into a
% stable, reference-tracking system (after).
figure
step(fwd,0:0.01:5)
hold on
step(T,0:0.01:5)
yline(1,'k:','HandleVisibility','off')
hold off
legend('Before (open forward path)','After (closed loop)','Interpreter','latex','FontSize',12,'Location','northwest')
title('Problem 3: Before vs. After Closing the Loop','Interpreter','latex','FontSize',16)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Try it yourself
% * In Problem 2, change the target |zeta| to 0.2 and recompute |b|:
%   notice how much less damping the system then needs.
% * In Problem 3, remove the integrator (|G2 = tf(5,[1 5])|) and see the
%   steady-state error stop being zero.
