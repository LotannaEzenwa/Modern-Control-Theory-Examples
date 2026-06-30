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

%% Before: The Uncompensated Locus Misses the Target
% Plot the uncompensated root locus and mark the desired dominant pole
% s_d (both conjugates). It does not lie on the locus, so no value of
% plain gain K can place a closed-loop pole there -- which is exactly why
% a compensator is needed.
figure
rlocus(G)
hold on
plot(real(sd),imag(sd),'rp','MarkerSize',14,'MarkerFaceColor','r')
plot(real(sd),-imag(sd),'rp','MarkerSize',14,'MarkerFaceColor','r')
hold off
legend('Uncompensated locus','Desired pole $s_d$','Interpreter','latex','FontSize',12)
title('Before: Desired Pole Is Off the Uncompensated Locus','Interpreter','latex','FontSize',18)
grid on

%% Angle Deficiency
% The open-loop transfer function has no finite zeros and a positive DC
% gain, so its phase at $s_d$ is $\angle G(s_d) = -\sum_i\angle(s_d-p_i)$.
% For $s_d$ to lie on the *compensated* locus, the lead network must add
% just enough phase that the total is $-180^\circ$:
%
% $\phi_{lead} = -180^\circ - \angle G(s_d)$.
[~,p] = tfdata(G,'v');
ol_poles = roots(p);
phase_G = -sum(angle(sd - ol_poles))*180/pi;   % angle of G(sd), degrees
phi_lead = mod((-180 - phase_G) + 360, 360);   % lead phase to add (deg)
fprintf('Open-loop phase at s_d = %.2f deg\n', phase_G)
fprintf('Phase the lead compensator must add = %.2f deg\n', phi_lead)

%% Placing the Compensator Zero and Pole
% A common design choice places the compensator zero directly beneath the
% desired pole's real part (zero at $s=\mathrm{Re}(s_d)$), then solves for
% the pole that supplies the remaining angle. With the zero at $-z_c$ the
% zero contributes $\angle(s_d+z_c)$, so the pole must contribute
% $\angle(s_d+p_c) = \angle(s_d+z_c) - \phi_{lead}$.
zc = -real(sd);                       % zero at s = Re(s_d) (left-half plane)
angle_zero = angle(sd + zc)*180/pi;
angle_pole = angle_zero - phi_lead;   % required angle from the compensator pole
pc = -real(sd) + imag(sd)/tand(angle_pole);   % pole at -pc on the real axis
fprintf('Lead compensator zero at s = %.4f\n', -zc)
fprintf('Lead compensator pole at s = %.4f\n', -pc)

Gc = tf([1 zc],[1 pc]);

%% Gain Selection via the Magnitude Condition
Kc = 1/abs(evalfr(Gc*G,sd));
fprintf('Compensator gain Kc = %.4f\n', Kc)

%% Verification
% Confirm the design: with the chosen $K_c$, a closed-loop pole should sit
% essentially on top of the desired location $s_d$.
T_check = feedback(Kc*Gc*G,1);
cl_poles = pole(T_check);
[~,idx] = min(abs(cl_poles - sd));
fprintf('Desired s_d = %.4f + %.4fj; nearest closed-loop pole = %.4f + %.4fj\n', ...
    real(sd), imag(sd), real(cl_poles(idx)), imag(cl_poles(idx)))

Gc_full = Kc*Gc;
G_comp = series(Gc_full,G);
T_comp = feedback(G_comp,1);
T_uncomp = feedback(G,1);

%% What I Did: The Compensator Reshapes the Locus
% Adding the lead pole/zero pulls the locus to the left so that it now
% passes through the desired pole s_d (marked again for reference).
figure
rlocus(G_comp)
hold on
plot(real(sd),imag(sd),'rp','MarkerSize',14,'MarkerFaceColor','r')
plot(real(sd),-imag(sd),'rp','MarkerSize',14,'MarkerFaceColor','r')
hold off
legend('Compensated locus','Desired pole $s_d$','Interpreter','latex','FontSize',12)
title('What Changed: Compensated Locus Now Passes Through $s_d$','Interpreter','latex','FontSize',17)
grid on

%% After: What Changed in the Time Response
% Side by side, the lead compensator speeds up the response and cuts the
% overshoot relative to the uncompensated closed loop.
figure
hold on
step(T_uncomp)
step(T_comp)
hold off
legend('Before (uncompensated)','After (lead-compensated)','Interpreter','latex','FontSize',14)
title('Lead Compensation: Step Response Before vs. After','Interpreter','latex','FontSize',18)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

info_before = stepinfo(T_uncomp);
info_after  = stepinfo(T_comp);
fprintf('Overshoot:     before = %.1f%%, after = %.1f%%\n', info_before.Overshoot, info_after.Overshoot)
fprintf('Settling time: before = %.3f s, after = %.3f s\n', info_before.SettlingTime, info_after.SettlingTime)

%% After: Where the Dominant Poles Moved
% The pole map makes the design goal explicit -- the dominant closed-loop
% poles move from the slow uncompensated location to the faster, better
% damped target near s_d.
figure
hold on
plot(real(pole(T_uncomp)),imag(pole(T_uncomp)),'bo','MarkerSize',9,'LineWidth',1.5)
plot(real(pole(T_comp)),imag(pole(T_comp)),'rx','MarkerSize',11,'LineWidth',1.5)
hold off
grid on
legend('Before (uncompensated poles)','After (compensated poles)','Interpreter','latex','FontSize',12)
title('Dominant Pole Movement (Before vs. After)','Interpreter','latex','FontSize',17)
ylabel('$\mathrm{Im}$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$\mathrm{Re}$','Interpreter','latex','FontSize',20)
