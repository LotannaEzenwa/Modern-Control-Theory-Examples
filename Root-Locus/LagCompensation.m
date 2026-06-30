%% Root-Locus Lag Compensation
% Ogata, Modern Control Engineering, Ch. 6: Lag Compensator Design via
% Root Locus
%
% A phase-lag compensator
%
% $G_c(s) = K_c\frac{s+z_c}{s+p_c}, \quad |z_c| > |p_c|$
%
% (zero farther from the origin than the pole, both close together and
% close to the origin) is used to improve *steady-state error* without
% significantly disturbing the transient response/dominant closed-loop
% poles already achieving a satisfactory $\zeta,\omega_n$ -- the
% opposite design goal from lead compensation.

%% Uncompensated Plant
% $G(s) = \frac{4}{s(s+1)(s+2)}$, already exhibiting acceptable
% transient response (the dominant poles are not being moved), but
% with insufficient velocity-error constant $K_v$.
G = tf(4,conv([1 0],conv([1 1],[1 2])));

%% Static Velocity Error Constant
% For a type-1 system, $K_v = \lim_{s\to0} sG(s)$, and the
% steady-state error to a unit ramp is $e_{ss}=1/K_v$.
s = tf('s');
Kv_uncomp = dcgain(s*G);
fprintf('Uncompensated Kv = %.4f, ess(ramp) = %.4f\n', Kv_uncomp, 1/Kv_uncomp)

%% Design Goal
% Suppose the specification requires $K_v \ge 20$ (i.e. $e_{ss}\le
% 0.05$) while leaving the dominant closed-loop poles essentially
% unchanged. The lag compensator increases the gain "seen" at DC by
% the ratio $z_c/p_c$ without changing the angle contribution near the
% dominant poles (since both $z_c,p_c$ are placed close to the origin,
% far from the dominant poles, their net angle contribution there is
% small).
Kv_desired = 20;
ratio = Kv_desired/Kv_uncomp;
fprintf('Required zc/pc ratio = %.4f\n', ratio)

% Place pc close to the origin (small, to avoid disturbing the locus
% near the dominant poles) and compute zc from the ratio.
pc = 0.05;
zc = ratio*pc;
fprintf('Lag compensator pole at s = %.4f\n', -pc)
fprintf('Lag compensator zero at s = %.4f\n', -zc)

Gc = tf([1 zc],[1 pc]);
G_comp = series(Gc,G);

Kv_comp = dcgain(s*G_comp);
fprintf('Compensated Kv = %.4f, ess(ramp) = %.4f\n', Kv_comp, 1/Kv_comp)

%% Verifying the Dominant Poles Are Largely Unchanged
T_uncomp = feedback(G,1);
T_comp = feedback(G_comp,1);

poles_uncomp = pole(T_uncomp)
poles_comp = pole(T_comp)

figure
hold on
step(T_uncomp)
step(T_comp)
hold off
legend('Uncompensated','Lag-Compensated','Interpreter','latex','FontSize',14)
title('Lag Compensation: Step Response Comparison','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% The lag compensator's pole-zero pair sits close together and near
% the origin, so it contributes negligible angle at the dominant
% closed-loop poles -- the transient response is nearly preserved
% while $K_v$ (and hence the ramp-tracking accuracy) improves by the
% factor $z_c/p_c$.
