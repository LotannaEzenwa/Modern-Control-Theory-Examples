%% Mathematical Models -- Introduction
% Ogata, Modern Control Engineering, Ch. 2: Mathematical Modeling of
% Control Systems
%
% A mathematical model of a dynamic system is the set of equations that
% represent its dynamics. For linear, time-invariant (LTI) systems
% governed by a constant-coefficient linear ODE
%
% $$a_n\frac{d^n y}{dt^n} + \dots + a_1\dot{y} + a_0 y =
%   b_m\frac{d^m u}{dt^m} + \dots + b_1\dot{u} + b_0 u$$
%
% the Laplace transform converts the ODE (with zero initial conditions)
% into an algebraic relationship between the transforms of the input and
% output, $U(s)$ and $Y(s)$.

%% The Laplace Transform
% The Laplace transform of a time function $f(t)$, $t \geq 0$, is
%
% $$F(s) = \mathcal{L}[f(t)] = \int_0^\infty f(t) e^{-st}\,dt$$
%
% Two properties make it the natural tool for LTI systems:
%
% * Differentiation: $\mathcal{L}[\dot f(t)] = sF(s) - f(0)$
% * Linearity: $\mathcal{L}[a f(t) + b g(t)] = aF(s) + bG(s)$
%
% Common transform pairs used throughout this repository:
%
% $$\mathcal{L}[1(t)] = \frac{1}{s}, \quad
%   \mathcal{L}[t] = \frac{1}{s^2}, \quad
%   \mathcal{L}[e^{-at}] = \frac{1}{s+a}, \quad
%   \mathcal{L}[\sin(\omega t)] = \frac{\omega}{s^2+\omega^2}$$

syms t s
f1 = heaviside(t);          % unit step
f2 = t;                     % unit ramp
f3 = exp(-2*t);             % decaying exponential
f4 = sin(3*t);              % sinusoid

F1 = laplace(f1,t,s);
F2 = laplace(f2,t,s);
F3 = laplace(f3,t,s);
F4 = laplace(f4,t,s);

disp('Step, ramp, exponential, sinusoid transforms:')
disp([F1 F2 F3 F4])

%% From ODE to Transfer Function
% With zero initial conditions, taking the Laplace transform of the
% governing ODE above and solving for the ratio $Y(s)/U(s)$ gives the
% *transfer function*
%
% $$G(s) = \frac{Y(s)}{U(s)} =
%   \frac{b_m s^m + \dots + b_1 s + b_0}{a_n s^n + \dots + a_1 s + a_0}$$
%
% As a first example, consider
%
% $$\ddot{y} + 3\dot{y} + 2y = u$$
%
% Taking the Laplace transform term by term: $s^2 Y(s) + 3sY(s) + 2Y(s)
% = U(s)$, so
%
% $$G(s) = \frac{1}{s^2+3s+2}$$

G = tf(1,[1 3 2]);
G

%%
% The poles of $G(s)$ are the roots of the denominator polynomial; they
% determine the natural (unforced) response of the system.
poles_G = pole(G)

%%
% A unit-step response confirms the system settles to its DC gain,
% $G(0) = 1/2$, consistent with the final value theorem
% $\lim_{t\to\infty} y(t) = \lim_{s\to 0} sG(s)\frac{1}{s} = G(0)$.
figure
step(G)
title('Step Response of $G(s)=\frac{1}{s^2+3s+2}$','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% The remaining files in this directory build up the physical systems
% (mechanical, electrical, fluid, thermal) whose governing equations
% produce transfer functions of this type, plus the block-diagram
% algebra used to combine them into full control systems.
