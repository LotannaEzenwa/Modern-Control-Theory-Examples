%% Second-Order System Response
% Ogata, Modern Control Engineering, Ch. 5: Second-Order Systems
%
% The standard second-order transfer function is
%
% $\frac{Y(s)}{R(s)} = \frac{\omega_n^2}{s^2+2\zeta\omega_n s+\omega_n^2}$
%
% where $\omega_n$ is the *undamped natural frequency* and $\zeta$ is
% the *damping ratio*. The closed-loop poles are
%
% $s = -\zeta\omega_n \pm j\omega_n\sqrt{1-\zeta^2}$
%
% Depending on $\zeta$, the system is:
%
% * *Overdamped* ($\zeta>1$): two distinct real poles, no overshoot.
% * *Critically damped* ($\zeta=1$): repeated real pole at $-\omega_n$.
% * *Underdamped* ($0<\zeta<1$): complex-conjugate poles, decaying
%   oscillation.
% * *Undamped* ($\zeta=0$): poles on the $j\omega$-axis, sustained
%   oscillation at $\omega_n$.

%% Family of Step Responses for Varying $\zeta$
wn = 1;
zetas = [0 0.2 0.4 0.6 0.8 1.0 1.5];
figure
hold on
for z = zetas
    Gz = tf(wn^2,[1 2*z*wn wn^2]);
    step(Gz, 0:0.01:15)
end
hold off
legend(arrayfun(@(z) sprintf('$\\zeta=%.1f$',z), zetas, 'UniformOutput', false), ...
    'Interpreter','latex','FontSize',12)
title('Second-Order Step Response vs. Damping Ratio','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Underdamped Transient-Response Specifications
% For the underdamped case ($0<\zeta<1$), the unit-step response is
%
% $y(t) = 1 - \frac{e^{-\zeta\omega_n t}}{\sqrt{1-\zeta^2}}
%   \sin(\omega_d t + \beta), \quad
%   \omega_d=\omega_n\sqrt{1-\zeta^2}, \quad
%   \beta=\cos^{-1}\zeta$
%
% From this, the standard transient specifications are derived:
%
% * *Rise time* (0% to 100%): $t_r \approx \frac{\pi-\beta}{\omega_d}$
% * *Peak time*: $t_p = \frac{\pi}{\omega_d}$
% * *Maximum overshoot*:
%   $M_p = e^{-\zeta\pi/\sqrt{1-\zeta^2}}$ (fraction of final value)
% * *2% Settling time*: $t_s \approx \frac{4}{\zeta\omega_n}$
zeta = 0.5; wn = 4;
G = tf(wn^2,[1 2*zeta*wn wn^2]);

wd = wn*sqrt(1-zeta^2);
beta = acos(zeta);
tr = (pi-beta)/wd;
tp = pi/wd;
Mp = exp(-zeta*pi/sqrt(1-zeta^2));
ts = 4/(zeta*wn);

fprintf('Rise time      tr = %.4f s\n', tr)
fprintf('Peak time      tp = %.4f s\n', tp)
fprintf('Max overshoot  Mp = %.2f%%\n', Mp*100)
fprintf('Settling time  ts = %.4f s (2%% criterion)\n', ts)

figure
step(G)
hold on
yline(1+Mp,'--')
yline(1,':')
xline(tp,'--')
hold off
title('Second-Order Underdamped Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Verification via MATLAB's stepinfo
% The Control System Toolbox's |stepinfo| computes these same
% specifications numerically from the simulated response, providing a
% check on the closed-form formulas above.
info = stepinfo(G)
