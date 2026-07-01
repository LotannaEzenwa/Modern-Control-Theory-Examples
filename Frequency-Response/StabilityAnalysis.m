%% Relative Stability: Gain Margin and Phase Margin
% *How much headroom is there before instability?*
%
% Ogata, _Modern Control Engineering_, Ch. 7.
%
% In this tutorial you will:
%
% * define gain margin and phase margin and read them with |margin|,
% * relate phase margin to the equivalent damping ratio, and
% * watch the margins shrink to zero as the loop gain rises.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% For a minimum-phase open-loop system whose Nyquist plot crosses the
% real axis to the right of $-1$, two scalar measures of *relative*
% stability (margin before instability) are defined:
%
% *Gain margin* $GM = \frac{1}{|G(j\omega_p)|}$ (often in dB:
% $GM_{dB}=-20\log_{10}|G(j\omega_p)|$), evaluated at the *phase
% crossover frequency* $\omega_p$ where $\angle G(j\omega_p)=-180^\circ$
% -- the factor by which the gain can increase before instability.
%
% *Phase margin* $PM = 180^\circ + \angle G(j\omega_c)$, evaluated at the
% *gain crossover frequency* $\omega_c$ where $|G(j\omega_c)|=1$ -- the
% additional phase lag that can be tolerated before instability.

%% Worked Example
% $G(s) = \frac{5}{s(s+1)(s+4)}$
% (Gain 5 sits comfortably below the critical gain of 20 for this plant,
% so the margins come out positive -- exactly the healthy case the next
% section interprets. Watch what happens to these numbers at $K=20$ in the
% gain sweep further down.)
G = tf(5,conv([1 0],conv([1 1],[1 4])));

[Gm,Pm,Wcp,Wcg] = margin(G);
GmdB = 20*log10(Gm);
fprintf('Gain margin = %.4f (%.2f dB) at wp = %.4f rad/s\n', Gm, GmdB, Wcp)
fprintf('Phase margin = %.4f deg at wc = %.4f rad/s\n', Pm, Wcg)

figure
margin(G)
title('Bode Plot with Gain/Phase Margin','Interpreter','latex','FontSize',20)
grid on

%% Interpreting the Margins
% Both margins positive ($GM>1$, i.e. $GM_{dB}>0$, and $PM>0^\circ$)
% indicates closed-loop stability; small positive margins indicate a
% lightly-damped, oscillatory closed-loop response, while large margins
% indicate sluggish but robust response. A practical rule of thumb (per
% Ogata) targets $PM\approx30^\circ\mathrm{--}60^\circ$ and
% $GM_{dB}>6$ dB.
T = feedback(G,1);
fprintf('Closed-loop poles:\n'); disp(pole(T))
fprintf('Closed-loop stable: %d\n', all(real(pole(T))<0))

figure
step(T)
title('Closed-Loop Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Phase Margin vs. Damping Ratio (Approximate Correlation)
% For a second-order-dominant system, a commonly used approximation
% relates phase margin (in degrees) to the equivalent damping ratio:
%
% $\zeta \approx \frac{PM}{100}$  (for $PM$ up to about $60^\circ$)
zeta_approx = Pm/100;
fprintf('Approximate equivalent damping ratio from PM: zeta = %.4f\n', zeta_approx)

%% Sweeping Gain: Margins Shrink Toward Instability
% As the loop gain rises the phase margin shrinks and eventually goes
% negative (closed-loop unstable). Print GM/PM across a range:
Ks = [5 20 60 96];
for k = Ks
    Gk = tf(k,conv([1 0],conv([1 1],[1 4])));
    [gm,pm] = margin(Gk);
    fprintf('K=%5.1f: GM=%.3f (%.2f dB), PM=%.2f deg\n', k, gm, 20*log10(gm), pm)
end

%% Before vs. After in the Time Domain
% The same trend as closed-loop step responses (using stable gains so the
% curves stay on screen): more gain -> less phase margin -> more overshoot
% and ringing, marching from "before" (well damped) toward instability.
Ks_stable = [5 12 18];
figure
hold on
for k = Ks_stable
    step(feedback(tf(k,conv([1 0],conv([1 1],[1 4]))),1), 0:0.02:15)
end
hold off
grid on
legend(arrayfun(@(k) sprintf('$K=%d$',k), Ks_stable,'UniformOutput',false), ...
    'Interpreter','latex','FontSize',12)
title('Closed-Loop Step as Gain Rises (Margin Shrinks)','Interpreter','latex','FontSize',16)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Try it yourself
% * Push a stable gain from 5 toward 20 and watch |margin| report the phase
%   margin shrinking toward zero as the step response rings harder.
% * Use the approximation |zeta ~ PM/100| to predict overshoot, then check
%   it against |stepinfo(feedback(Gk,1))|.
