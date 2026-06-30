%% Ziegler-Nichols PID Tuning
% *Two classic recipes for tuning PID without a model.*
%
% Ogata, _Modern Control Engineering_, Ch. 8.
%
% In this tutorial you will:
%
% * apply the *reaction-curve* (first) method from an open-loop step,
% * apply the *ultimate-gain* (second) method from sustained oscillation, and
% * compare the two tunings on the closed-loop step response.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% Ziegler and Nichols proposed two empirical rules for choosing
% $K_p,T_i,T_d$ that give roughly a 25% overshoot decay ratio, without
% needing a full mathematical model of the plant.

%% First Method: Reaction Curve (Open-Loop Step Response)
% Applicable to plants whose open-loop unit-step response is
% S-shaped (no integration, no dominant complex poles) -- characterized
% by a *delay time* $L$ and a *time constant* $T$, read from the point of
% inflection's tangent line.
%
% Plant: $G(s)=\frac{1}{(s+1)(s+2)(s+3)}$ (no integrator, S-shaped step
% response).
G = tf(1,conv(conv([1 1],[1 2]),[1 3]));
[y,t] = step(G,0:0.001:10);

% Find the inflection point (max slope) and its tangent line.
dy = gradient(y,t);
[maxslope,idx] = max(dy);
t_infl = t(idx); y_infl = y(idx);

% Tangent line: y = y_infl + maxslope*(t - t_infl); intersects y=0 at L,
% and reaches the final value at L+T.
yfinal = dcgain(G);
L = t_infl - y_infl/maxslope;
T_rc = (yfinal - y_infl)/maxslope + t_infl - L;
fprintf('Reaction curve: L = %.4f, T = %.4f\n', L, T_rc)

figure
plot(t,y,'b','LineWidth',1.5)
hold on
tang = y_infl + maxslope*(t-t_infl);
plot(t,tang,'r--')
yline(yfinal,'k:')
xline(L,'g:'); xline(L+T_rc,'g:')
hold off
ylim([0 1.2*yfinal])
title('Reaction-Curve Method: $L$ and $T$','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% Ziegler-Nichols first-method tuning table:
%
% P:   $K_p = T/L$
%
% PI:  $K_p = 0.9T/L,\ T_i = L/0.3$
%
% PID: $K_p = 1.2T/L,\ T_i = 2L,\ T_d = 0.5L$
Kp_zn1 = 1.2*T_rc/L;
Ti_zn1 = 2*L;
Td_zn1 = 0.5*L;
fprintf('ZN Method 1 PID: Kp=%.4f, Ti=%.4f, Td=%.4f\n', Kp_zn1, Ti_zn1, Td_zn1)

Gc1 = tf(Kp_zn1*[Td_zn1 1 1/Ti_zn1],[1 0]);
T1 = feedback(Gc1*G,1);
figure
step(T1)
title('ZN Method 1: Closed-Loop PID Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Second Method: Ultimate Gain / Ultimate Period (Closed-Loop)
% Applicable more generally: with only proportional control in the loop,
% raise $K_p$ until the closed-loop response is a sustained oscillation
% (marginal stability). That gain is $K_{cr}$, and the oscillation
% period is $P_{cr}$.
%
% For $G(s)=\frac{1}{(s+1)(s+2)(s+3)}$, the characteristic equation under
% proportional gain $K$ is $(s+1)(s+2)(s+3)+K=0$, i.e.
% $s^3+6s^2+11s+(6+K)=0$. By Routh's array, the marginal-stability gain
% is found from the $s^1$ row: $\frac{6\times11-(6+K)}{6}=0 \Rightarrow K=60$.
Kcr = 60;
fprintf('Ultimate gain Kcr = %.4f\n', Kcr)

% At K=Kcr the auxiliary equation 6s^2+(6+Kcr)=0 gives the oscillation
% frequency.
omega_cr = sqrt((6+Kcr)/6);
Pcr = 2*pi/omega_cr;
fprintf('Ultimate period Pcr = %.4f s (omega_cr = %.4f rad/s)\n', Pcr, omega_cr)

T_marginal = feedback(Kcr*G,1);
figure
impulse(T_marginal,0:0.01:10)
title('Sustained Oscillation at $K=K_{cr}$ (Verifying $P_{cr}$)','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% Ziegler-Nichols second-method tuning table:
%
% P:   $K_p = 0.5K_{cr}$
%
% PI:  $K_p = 0.45K_{cr},\ T_i = P_{cr}/1.2$
%
% PID: $K_p = 0.6K_{cr},\ T_i = 0.5P_{cr},\ T_d = 0.125P_{cr}$
Kp_zn2 = 0.6*Kcr;
Ti_zn2 = 0.5*Pcr;
Td_zn2 = 0.125*Pcr;
fprintf('ZN Method 2 PID: Kp=%.4f, Ti=%.4f, Td=%.4f\n', Kp_zn2, Ti_zn2, Td_zn2)

Gc2 = tf(Kp_zn2*[Td_zn2 1 1/Ti_zn2],[1 0]);
T2 = feedback(Gc2*G,1);

figure
hold on
step(T1)
step(T2)
hold off
legend('Method 1 (Reaction Curve)','Method 2 (Ultimate Gain)','Interpreter','latex','FontSize',14)
title('Ziegler-Nichols: Method 1 vs. Method 2 PID Tuning','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Before vs. After: Uncontrolled Plant vs. ZN-Tuned PID
% The payoff: compared with the bare plant's open-loop step (slow, large
% steady-state error), the Ziegler-Nichols PID tracks the reference with
% integral action and a reasonable transient.
figure
step(G,0:0.01:10)
hold on
step(T2,0:0.01:10)
yline(1,'k:','HandleVisibility','off')
hold off
legend('Before (open-loop plant)','After (ZN Method 2 PID)','Interpreter','latex','FontSize',13)
title('Ziegler-Nichols: Before vs. After','Interpreter','latex','FontSize',17)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
