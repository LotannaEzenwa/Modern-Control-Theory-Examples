%% Root-Locus Lag Compensation
% *Improving steady-state accuracy without disturbing the transient.*
%
% Ogata, _Modern Control Engineering_, Ch. 6.
%
% In this tutorial you will:
%
% * size a lag compensator from a velocity-error-constant ($K_v$) spec,
% * place its pole/zero pair near the origin to preserve the dominant poles, and
% * confirm the transient is unchanged while ramp tracking improves.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
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

%% Before: The Uncompensated Ramp-Tracking Error
% Lag compensation targets steady-state accuracy, so the telling "before"
% picture is the response to a unit ramp: the uncompensated output lags
% the reference by a constant ess = 1/Kv.
t_r = 0:0.01:10;
y_uncomp_ramp = lsim(feedback(G,1),t_r,t_r);
figure
plot(t_r,t_r,'k--',t_r,y_uncomp_ramp,'b','LineWidth',1.2)
legend('Reference $r(t)=t$','Uncompensated output','Interpreter','latex','FontSize',12,'Location','northwest')
title('Before: Steady-State Ramp Lag (Uncompensated)','Interpreter','latex','FontSize',18)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

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

%% After: What Changed -- Ramp Tracking Improves
% The same ramp input, now with the compensator: raising Kv by the factor
% zc/pc shrinks the steady-state lag dramatically.
y_comp_ramp = lsim(T_comp,t_r,t_r);
figure
plot(t_r,t_r,'k--',t_r,y_uncomp_ramp,'b',t_r,y_comp_ramp,'r','LineWidth',1.2)
legend('Reference','Before (uncompensated)','After (lag-compensated)', ...
    'Interpreter','latex','FontSize',12,'Location','northwest')
title('After: Ramp Tracking Before vs. After','Interpreter','latex','FontSize',18)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% What Did NOT Change: The Transient Response
% By design the dominant poles barely move, so the step (transient)
% response is almost identical -- the whole point of lag compensation is
% to fix steady-state accuracy without disturbing the transient.
figure
hold on
step(T_uncomp)
step(T_comp)
hold off
legend('Before (uncompensated)','After (lag-compensated)','Interpreter','latex','FontSize',14)
title('Transient Response Is Essentially Unchanged','Interpreter','latex','FontSize',17)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Where the Dominant Poles Moved (Hardly at All)
figure
hold on
plot(real(poles_uncomp),imag(poles_uncomp),'bo','MarkerSize',9,'LineWidth',1.5)
plot(real(poles_comp),imag(poles_comp),'rx','MarkerSize',11,'LineWidth',1.5)
hold off
grid on
legend('Before','After','Interpreter','latex','FontSize',12)
title('Dominant Pole Map: Before vs. After Lag Compensation','Interpreter','latex','FontSize',16)
ylabel('$\mathrm{Im}$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$\mathrm{Re}$','Interpreter','latex','FontSize',20)

%%
% Summary: Kv (and ramp accuracy) improves by the factor zc/pc while the
% dominant closed-loop poles -- and hence the transient response -- are
% left essentially unchanged.

%% Try it yourself
% * Demand |Kv_desired = 50| instead of 20 and notice the zero/pole ratio
%   (and the ramp accuracy) increase, with the transient still untouched.
% * Move the pole out to |pc = 0.2| and watch it start to disturb the
%   dominant poles -- a lag pole/zero pair must stay near the origin.
