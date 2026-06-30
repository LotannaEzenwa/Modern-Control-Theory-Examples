%% Frequency-Response -- Worked Problems
% *Practice: read margins, find the max gain for a spec, and design a lead.*
%
% Ogata, _Modern Control Engineering_, Ch. 7 (end-of-chapter style).
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.

%% Problem 1: Find Gain/Phase Margins and Assess Stability
% For $G(s) = \frac{K}{s(s+2)(s+10)}$ with $K=200$, find the gain and
% phase margins and determine whether the closed-loop system is stable.
K1 = 200;
G1 = tf(K1,conv([1 0],conv([1 2],[1 10])));
[Gm1,Pm1,Wcp1,Wcg1] = margin(G1);
fprintf('Problem 1: GM = %.4f (%.2f dB) at wp=%.4f, PM = %.2f deg at wc=%.4f\n', ...
    Gm1, 20*log10(Gm1), Wcp1, Pm1, Wcg1)
fprintf('Stable (both margins positive): %d\n', (Gm1>1) && (Pm1>0))

figure
margin(G1)
title('Problem 1: Gain/Phase Margins','Interpreter','latex','FontSize',20)
grid on

%% Problem 2: Maximum Gain for a Phase-Margin Specification
% For the same plant family $G(s)=\frac{K}{s(s+2)(s+10)}$, find the
% largest $K$ for which $PM \ge 45^\circ$.
K_range = 1:1:300;
PMs = zeros(size(K_range));
for i = 1:numel(K_range)
    Gk = tf(K_range(i),conv([1 0],conv([1 2],[1 10])));
    [~,PMs(i)] = margin(Gk);
end
idx_ok = find(PMs>=45,1,'last');
K_max = K_range(idx_ok);
fprintf('Problem 2: Largest K with PM>=45 deg is K = %d (PM = %.2f deg)\n', ...
    K_max, PMs(idx_ok))

figure
plot(K_range,PMs)
hold on
yline(45,'r--')
hold off
title('Problem 2: Phase Margin vs. Gain','Interpreter','latex','FontSize',20)
ylabel('$PM$ (deg)','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$K$','Interpreter','latex','FontSize',20)
grid on

%% Problem 3: Lead Compensator to Meet a Phase-Margin Spec
% For $G(s)=\frac{10}{s(s+1)}$, design a lead compensator so that the
% phase margin is at least $40^\circ$ while preserving the existing
% velocity-error constant $K_v=10$.
G3 = tf(10,[1 1 0]);
PM_des3 = 40;
[~,PM1_3,~,wc1_3] = margin(G3);
fprintf('Problem 3: Uncompensated PM = %.2f deg at wc=%.2f\n', PM1_3, wc1_3)

epsilon3 = 10;
phi_max3 = (PM_des3 - PM1_3 + epsilon3)*pi/180;
alpha3 = (1+sin(phi_max3))/(1-sin(phi_max3));

w_sweep3 = logspace(-1,2,2000);
[mag3,~] = bode(G3,w_sweep3);
mag_dB3 = 20*log10(squeeze(mag3));
mag_needed_dB3 = -10*log10(alpha3);
[~,idx3] = min(abs(mag_dB3 - mag_needed_dB3));
wm3 = w_sweep3(idx3);
T3 = 1/(wm3*sqrt(alpha3));

Gc3 = tf([alpha3*T3 1],[T3 1]);
G3_comp = series(Gc3,G3);
[~,PM3_comp,~,wc3_comp] = margin(G3_comp);
fprintf('Problem 3: alpha=%.4f, T=%.4f, Compensated PM = %.2f deg at wc=%.2f\n', ...
    alpha3, T3, PM3_comp, wc3_comp)

T3_uncomp = feedback(G3,1);
T3_comp_cl = feedback(G3_comp,1);
figure
hold on
step(T3_uncomp)
step(T3_comp_cl)
hold off
legend('Uncompensated','Lead-Compensated','Interpreter','latex','FontSize',14)
title('Problem 3: Lead-Compensated Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
