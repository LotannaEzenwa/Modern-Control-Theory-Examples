%% First-Order System Response
% Ogata, Modern Control Engineering, Ch. 5: First-Order Systems
%
% A first-order system has the transfer function
%
% $\frac{Y(s)}{R(s)} = \frac{1}{Ts+1}$
%
% where $T$ is the system's *time constant*. Physically this is the
% same form derived for the RC circuit, the liquid-level tank, and the
% thermometer in |Mathematical Models/|.

%% Unit-Step Response Derivation
% With $R(s)=1/s$:
%
% $Y(s) = \frac{1}{Ts+1}\cdot\frac{1}{s} = \frac{1}{s} - \frac{T}{Ts+1}$
%
% Taking the inverse Laplace transform:
%
% $y(t) = 1 - e^{-t/T}, \quad t\ge 0$
%
% so $y(t)\to 1$ as $t\to\infty$ with no overshoot -- a first-order
% system's step response is always monotonic.
T = 2;
G = tf(1,[T 1]);

figure
[ys,ts] = step(G);
plot(ts, ones(size(ts)),'k--', ts, ys,'b','LineWidth',1.3)
hold on
plot(T, 1-exp(-1),'ro','MarkerSize',9,'MarkerFaceColor','r')
xline(T,'r:')
hold off
legend('Input step (before)','Output $y(t)$ (after)','$63.2\%$ at $t=T$', ...
    'Interpreter','latex','FontSize',11,'Location','southeast')
title('First-Order Step: Input vs. Output (time constant $T$)','Interpreter','latex','FontSize',15)
ylabel('amplitude','Interpreter','latex','FontSize',16)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% The 63.2% Time-Constant Rule
% At $t=T$: $y(T) = 1-e^{-1}\approx 0.632$. At $t=2T$,
% $y(2T)=1-e^{-2}\approx 0.865$; at $t=3T$, $y(3T)=1-e^{-3}\approx
% 0.950$; at $t=4T$, $y(4T)\approx 0.982$. By convention the system is
% considered to have settled after about $4T$ (the 2% settling-time
% rule).
[y,t] = step(G);
y_at_T  = interp1(t,y,T);
y_at_2T = interp1(t,y,2*T);
y_at_3T = interp1(t,y,3*T);
y_at_4T = interp1(t,y,4*T);
fprintf('y(T)  = %.4f (expect ~0.632)\n', y_at_T)
fprintf('y(2T) = %.4f (expect ~0.865)\n', y_at_2T)
fprintf('y(3T) = %.4f (expect ~0.950)\n', y_at_3T)
fprintf('y(4T) = %.4f (expect ~0.982)\n', y_at_4T)

%% Slope at the Origin
% Differentiating $y(t)=1-e^{-t/T}$ gives $\dot{y}(0)=1/T$: the
% tangent line drawn at $t=0$ reaches the final value $y=1$ exactly at
% $t=T$, a graphical method for reading $T$ off an experimental step
% response.
tangent = t/T;
figure
hold on
plot(t,y)
plot(t,tangent,'--')
ylim([0 1.2])
hold off
legend('$y(t)$','Tangent at $t=0$','Interpreter','latex','FontSize',14,'Location','southeast')
title('Graphical Time-Constant Estimation','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Unit-Ramp and Unit-Impulse Response
% For a unit ramp input, $Y(s) = \frac{1}{s^2(Ts+1)}$, giving
% $y(t) = t - T + Te^{-t/T}$; the steady-state error to a ramp is a
% constant lag of $T$ seconds. For a unit impulse,
% $y(t) = \frac{1}{T}e^{-t/T}$ -- a decaying exponential starting at
% $1/T$.
figure
subplot(2,1,1)
t2 = 0:0.01:10;
y_ramp = t2 - T + T*exp(-t2/T);
plot(t2,y_ramp)
title('First-Order Ramp Response','Interpreter','latex','FontSize',16)
ylabel('$y(t)$','Interpreter','latex','FontSize',16)
set(get(gca, 'YLabel'), 'Rotation', 0)

subplot(2,1,2)
impulse(G)
title('First-Order Impulse Response','Interpreter','latex','FontSize',16)
ylabel('$y(t)$','Interpreter','latex','FontSize',16)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',16)
