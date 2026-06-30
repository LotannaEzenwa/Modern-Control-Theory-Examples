%% Frequency-Domain Compensator Design
% Ogata, Modern Control Engineering, Ch. 7-8: Lead/Lag Compensation via
% the Frequency Response
%
% This file parallels the root-locus compensator designs in
% |Root-Locus/LeadCompensation.m| and |Root-Locus/LagCompensation.m|, but
% works directly with phase margin and Bode plots instead of pole/zero
% geometry in the $s$-plane.

%% Lead Compensator: Phase-Margin Specification
% A phase-lead compensator $G_c(s)=K_c\frac{1+\alpha Ts}{1+Ts}$,
% $\alpha>1$, adds positive phase. The maximum phase added,
%
% $\phi_{max} = \sin^{-1}\frac{\alpha-1}{\alpha+1}$
%
% occurs at $\omega_m=\frac{1}{T\sqrt{\alpha}}$, where the magnitude
% contribution is $10\log_{10}\alpha$ dB. Design procedure:
%
% # Find $K$ to satisfy any steady-state error spec; plot the
%   uncompensated Bode diagram.
% # Measure the uncompensated phase margin $PM_1$ at the gain crossover.
% # Compute the required additional phase
%   $\phi_{max}=PM_{desired}-PM_1+\epsilon$ (extra $\epsilon\approx5$-$12^\circ$
%   to allow for the crossover frequency shift).
% # Solve for $\alpha = \frac{1+\sin\phi_{max}}{1-\sin\phi_{max}}$.
% # Find the new crossover $\omega_m$ where the uncompensated magnitude
%   equals $-10\log_{10}\alpha$ dB; set $T=\frac{1}{\omega_m\sqrt{\alpha}}$.

%% Worked Example
% $G(s)=\frac{4}{s(s+2)}$, design spec: $PM\ge50^\circ$.
G = tf(4,[1 2 0]);

PM_desired = 50;
[~,PM1,~,wc1] = margin(G);
fprintf('Uncompensated PM = %.2f deg at wc = %.2f rad/s\n', PM1, wc1)

epsilon = 8;  % extra phase margin to compensate for crossover shift
phi_max = (PM_desired - PM1 + epsilon)*pi/180;
alpha = (1+sin(phi_max))/(1-sin(phi_max));
fprintf('Required phi_max = %.2f deg, alpha = %.4f\n', phi_max*180/pi, alpha)

% Magnitude needed from compensator at wm: -10*log10(alpha) dB.
% Find wm where |G(jw)| = -10*log10(alpha) dB.
mag_needed_dB = -10*log10(alpha);
w_sweep = logspace(-1,2,2000);
[mag,~] = bode(G,w_sweep);
mag_dB = 20*log10(squeeze(mag));
[~,idx] = min(abs(mag_dB - mag_needed_dB));
wm = w_sweep(idx);
T_lead = 1/(wm*sqrt(alpha));
fprintf('wm = %.4f rad/s, T = %.4f\n', wm, T_lead)

Gc = tf([alpha*T_lead 1],[T_lead 1]);
Kc = 1/abs(evalfr(Gc,1j*0))*1;  % unity DC gain on the compensator network itself
Gc_full = Kc*Gc;

G_comp = series(Gc_full,G);
[~,PM_comp,~,wc_comp] = margin(G_comp);
fprintf('Compensated PM = %.2f deg at wc = %.2f rad/s\n', PM_comp, wc_comp)

figure
hold on
bode(G)
bode(G_comp)
hold off
legend('Uncompensated','Lead-Compensated','Interpreter','latex','FontSize',14)
title('Frequency-Domain Lead Compensation','Interpreter','latex','FontSize',20)
grid on

T_uncomp = feedback(G,1);
T_comp = feedback(G_comp,1);
figure
hold on
step(T_uncomp)
step(T_comp)
hold off
legend('Uncompensated','Lead-Compensated','Interpreter','latex','FontSize',14)
title('Step Response: Frequency-Domain Lead Design','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Lag Compensator: Phase-Margin Specification (Brief Note)
% A phase-lag compensator $G_c(s)=K_c\frac{1+Ts}{1+\beta Ts}$, $\beta>1$,
% is placed so its corner frequencies sit roughly a decade below the new
% gain crossover, leaving phase nearly unaffected there while attenuating
% magnitude by $20\log_{10}\beta$ dB to pull the crossover down to a
% frequency where the existing phase already meets the margin spec --
% the frequency-domain dual of |Root-Locus/LagCompensation.m|, which
% pursued the same goal (steady-state error improvement without
% disturbing the dominant transient response) via pole/zero placement
% instead of a magnitude-attenuation argument.
