%% Higher-Order System Response and Dominant Poles
% Ogata, Modern Control Engineering, Ch. 5: Higher-Order Systems
%
% A system with three or more poles generally has no simple closed
% form for its transient specifications. However, if a pair of
% complex-conjugate poles lies much closer to the $j\omega$-axis than
% the remaining poles (a rule of thumb is a real-part ratio of 5:1 or
% more), that pair *dominates* the transient response, and the system
% behaves approximately like the second-order system formed by those
% poles alone -- the remaining "fast" poles decay quickly and
% contribute little after the initial transient.

%% Exact Third-Order System
% Consider
%
% $G(s) = \frac{20}{(s^2+2s+5)(s+10)}$
%
% with poles at $s=-1\pm j2$ (dominant pair, $\zeta\omega_n=1$) and
% $s=-10$ (fast pole, $5\times$ farther from the axis).
num = 20;
den = conv([1 2 5],[1 10]);
G_exact = tf(num,den);
poles_exact = pole(G_exact)

%% Dominant Second-Order Approximation
% Matching DC gain, the second-order approximation built from only the
% dominant pair is
%
% $G_{approx}(s) = \frac{K}{s^2+2s+5}, \quad
%   K = G_{exact}(0)\times 5$
%
% chosen so that $G_{approx}(0) = G_{exact}(0)$.
dc_exact = dcgain(G_exact);
K = dc_exact*5;
G_approx = tf(K,[1 2 5]);

fprintf('DC gain exact   = %.4f\n', dc_exact)
fprintf('DC gain approx  = %.4f\n', dcgain(G_approx))

figure
hold on
step(G_exact)
step(G_approx)
hold off
legend('Exact third-order','Dominant-pole approximation','Interpreter','latex','FontSize',14)
title('Dominant-Pole Approximation vs. Exact Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% The two curves nearly overlap after the brief initial transient
% contributed by the fast pole at $s=-10$, confirming the dominant-pole
% approximation is reasonable here.

%% When the Approximation Breaks Down
% If instead the "extra" pole is comparable in speed to the dominant
% pair (e.g. at $s=-3$, only $3\times$ the dominant real part), it
% contributes meaningfully to the transient and the second-order
% approximation is poor.
den_close = conv([1 2 5],[1 3]);
G_close = tf(dc_exact*15, den_close);  % rescaled for same DC gain as G_exact

figure
hold on
step(G_close)
step(G_approx)
hold off
legend('Exact (close extra pole at $s=-3$)','Dominant-pole approximation','Interpreter','latex','FontSize',12)
title('Approximation Breakdown: Non-Dominant Extra Pole','Interpreter','latex','FontSize',18)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
