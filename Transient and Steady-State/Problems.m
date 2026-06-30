%% Transient and Steady-State -- Worked Problems
% Ogata, Modern Control Engineering, Ch. 5: end-of-chapter style
% stability and transient-response problems.

%% Problem 1: Routh Stability Range
% A unity-feedback system has forward-path transfer function
%
% $G(s) = \frac{K}{s(s+2)(s+4)}$
%
% Find the range of $K>0$ for closed-loop stability.
%
% Characteristic equation: $s(s+2)(s+4)+K=0 \Rightarrow
%   s^3+6s^2+8s+K=0$
%
% Routh array:
%
%   s^3 |  1     8
%   s^2 |  6     K
%   s^1 |  (48-K)/6
%   s^0 |  K
%
% Requiring all first-column entries positive: $K>0$ and
% $48-K>0 \Rightarrow K<48$. So $0<K<48$.
syms s K
charpoly1 = expand(s*(s+2)*(s+4)+K)

Ks = [10 30 47.9 48 48.1];
for k = Ks
    p = roots([1 6 8 k]);
    fprintf('K = %5.1f -> max real part = %8.4f\n', k, max(real(p)))
end

%% Problem 2: Marginal Stability and Sustained Oscillation
% Using the boundary $K=48$ from Problem 1, the system is marginally
% stable; the row of zeros at $s^1$ indicates a pair of poles on the
% $j\omega$-axis. The auxiliary polynomial is formed from the $s^2$
% row: $6s^2+K=0 \Rightarrow s^2 = -K/6 \Rightarrow
%   s=\pm j\sqrt{K/6}$.
K_marginal = 48;
p_marginal = roots([1 6 8 K_marginal]);
disp('Closed-loop poles at K = 48:'); disp(p_marginal)
omega_pred = sqrt(K_marginal/6);
fprintf('Auxiliary-polynomial predicted omega = %.4f rad/s\n', omega_pred)

G_marginal = tf(K_marginal,[1 6 8 0]);
T_marginal = feedback(G_marginal,1);
figure
impulse(T_marginal, 0:0.01:10)
title('Problem 2: Sustained Oscillation at $K=48$','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Problem 3: Transient Specs from a Gain Choice
% For the same plant family, choose $K=20$ (well inside the stable
% range found in Problem 1) and find the closed-loop step-response
% specifications.
K3 = 20;
G3 = tf(K3,[1 6 8 0]);
T3 = feedback(G3,1);
poles_3 = pole(T3)

info3 = stepinfo(T3)

figure
step(T3)
title('Problem 3: Step Response at $K=20$','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)
