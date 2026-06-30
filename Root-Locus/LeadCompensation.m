%% Root-Locus Lead Compensation
% Ogata, Modern Control Engineering, Ch. 6: Lead Compensator Design via
% Root Locus
%
% A phase-lead compensator
%
% $G_c(s) = K_c\frac{s+z_c}{s+p_c}, \quad |z_c| < |p_c|$
%
% adds positive phase near the desired closed-loop pole location,
% "pulling" the root locus to the left and improving transient
% response (faster rise/settling time, reduced overshoot) at the cost
% of some bandwidth/noise-sensitivity increase.

%% Uncompensated Plant
% $G(s) = \frac{4}{s(s+2)}$, with a design specification of
% $\zeta=0.5$ (about 16% overshoot) and a faster settling time than
% the uncompensated system provides.
G = tf(4,[1 2 0]);

zeta = 0.5;
% Desired dominant closed-loop poles for a given settling time spec,
% e.g. ts = 1 s (4/(zeta*wn) = 1 => wn = 8)
wn_desired = 8;
sd = -zeta*wn_desired + 1j*wn_desired*sqrt(1-zeta^2);
fprintf('Desired dominant pole: %.4f + %.4fj\n', real(sd), imag(sd))

%% Angle Deficiency
% Evaluate the angle contributed by the uncompensated open-loop poles
% at $s_d$; the lead compensator must supply the remaining angle to
% satisfy the angle condition ($180^\circ$ total).
[~,p] = tfdata(G,'v');
ol_poles = roots(p);
angle_poles = sum(angle(sd - ol_poles))*180/pi;
angle_deficiency = 180 - (180 - angle_poles);   % angle that must be added by Gc
angle_deficiency = mod(180 - mod(angle_poles,360) + 360,360);
fprintf('Angle contributed by uncompensated poles = %.2f deg\n', angle_poles)
fprintf('Angle the lead compensator must add = %.2f deg\n', angle_deficiency)

%% Placing the Compensator Zero and Pole
% A common design choice places the compensator zero directly beneath
% (or near) the desired pole's real part to cancel some of the
% existing dynamics, then solves for the pole location that supplies
% the required angle contribution.
zc = real(sd);                      % place zero at the desired pole's real part
angle_zero = angle(sd - (-zc))*180/pi;

% angle from pole pc must equal: angle_zero - angle_deficiency (graphically)
% Solve for pc on the real axis using the geometry: the angle from pc
% to sd must close the angle condition.
target_angle_from_pc = angle_zero - angle_deficiency;
% angle(sd - (-pc)) = target_angle_from_pc (deg) -- solve for pc
pc_candidates = -real(sd) - imag(sd)/tand(target_angle_from_pc);
pc = pc_candidates;
fprintf('Lead compensator zero at s = %.4f\n', -zc)
fprintf('Lead compensator pole at s = %.4f\n', -pc)

Gc = tf([1 zc],[1 pc]);

%% Gain Selection via the Magnitude Condition
Kc = 1/abs(evalfr(Gc*G,sd));
fprintf('Compensator gain Kc = %.4f\n', Kc)

Gc_full = Kc*Gc;
G_comp = series(Gc_full,G);
T_comp = feedback(G_comp,1);
T_uncomp = feedback(G,1);

figure
hold on
step(T_uncomp)
step(T_comp)
hold off
legend('Uncompensated','Lead-Compensated','Interpreter','latex','FontSize',14)
title('Lead Compensation: Step Response Comparison','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

figure
rlocus(G_comp)
title('Compensated Root Locus','Interpreter','latex','FontSize',20)
grid on
