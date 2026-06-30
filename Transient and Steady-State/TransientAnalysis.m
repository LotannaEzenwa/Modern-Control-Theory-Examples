%% Transient and Steady-State Response Analysis
% Ogata, Modern Control Engineering, Ch. 5: Transient and Steady-State
% Response Analysis
%
% The time response of a control system is split into two parts:
%
% $y(t) = y_{transient}(t) + y_{steady-state}(t)$
%
% The transient part decays toward zero (for a stable system) and
% describes how the system moves from its initial condition to its
% final value; the steady-state part is what remains as
% $t\to\infty$. To compare systems on a common basis, we drive them
% with standard *test signals* rather than arbitrary inputs, since a
% system's response to any input can in principle be constructed from
% its response to these elementary signals.

%% Standard Test Signals
% * *Step*: $r(t) = R\cdot u_s(t)$, models a sudden, sustained change
%   in reference -- $R(s) = R/s$.
% * *Ramp*: $r(t) = Rt\cdot u_s(t)$, models a constant-velocity
%   reference -- $R(s) = R/s^2$.
% * *Impulse*: $r(t) = R\delta(t)$, an idealized instantaneous
%   disturbance -- $R(s) = R$.
% * *Parabolic*: $r(t) = \frac{R}{2}t^2\cdot u_s(t)$, a constant
%   acceleration reference -- $R(s) = R/s^3$.
%
% These are related by integration/differentiation in time, which
% corresponds to dividing/multiplying by $s$ in the Laplace domain.

%% Example System
% Consider a simple first-order plant with unity feedback,
% $G(s) = \frac{1}{s+1}$.
G = tf(1,[1 1]);
T = feedback(G,1);

%% Step Response
% MATLAB's |step| command applies a unit step input ($R(s)=1/s$) and
% plots $y(t)$.
figure
step(T)
title('Unit Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Ramp Response
% There is no dedicated |ramp| command; a unit ramp is simulated
% directly via |lsim| with a ramp input vector.
t = 0:0.01:10;
r_ramp = t;                      % unit ramp input r(t) = t
y_ramp = lsim(T,r_ramp,t);

figure
plot(t,r_ramp,'--',t,y_ramp,'-')
legend('Input $r(t)=t$','Output $y(t)$','Interpreter','latex','FontSize',14,'Location','northwest')
title('Unit Ramp Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% The steady-state ramp-following error is the vertical gap between
% the two lines as $t\to\infty$; for this type-0 closed loop it grows
% without bound, consistent with the static error coefficients
% discussed in |RouthStability.m| and the Root-Locus design files.

%% Impulse Response
% |impulse| applies $R(s)=1$; note this is exactly $\dot{y}_{step}(t)$,
% the time-derivative of the step response, illustrating the
% differentiation relationship between the test signals.
figure
impulse(T)
title('Unit Impulse Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)
