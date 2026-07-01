%% Digital Control I: Sampling, Zero-Order Hold, and Discretization
% *Turning a continuous plant into a model a computer can control.*
%
% Ogata, _Modern Control Engineering_, chapters on digital control.
%
% A digital controller sees the plant only at the sampling instants
% $t=kT$, and holds its command constant between samples through a
% *zero-order hold* (ZOH). Before we can design anything, we need a
% discrete-time model of the plant. In this tutorial you will:
%
% * sample a continuous plant and build its discrete model with |c2d|,
% * compare the *zero-order-hold* and *Tustin (bilinear)* discretizations,
% * see how continuous poles map to discrete poles through $z=e^{sT}$, and
% * watch accuracy degrade as the sample time $T$ grows.
%
% Run with |publish('Intro.m')|, or step through with *Ctrl+Enter*.

%% The continuous plant
% We start from a lightly damped second-order plant
%
% $$ G(s) = \frac{1}{s^2 + 0.6\,s + 1}. $$
G = tf(1,[1 0.6 1]);

%% Sampling with a zero-order hold
% The ZOH-equivalent discrete model |Gd| reproduces the continuous step
% response *exactly at the sampling instants*. Choose a sample time |T|
% short compared with the plant's natural period.
T  = 0.5;                  % sample time (seconds)
Gd = c2d(G, T, 'zoh');     % zero-order-hold discretization

figure
step(G,'b', Gd,'r--')
legend('Continuous $G(s)$','ZOH discrete $G(z)$','Interpreter','latex','FontSize',12)
title(sprintf('Step Response: Continuous vs. ZOH ($T=%.2f$ s)',T), ...
    'Interpreter','latex','FontSize',15)

%% Where did the poles go? The z = e^{sT} map
% Sampling maps each continuous pole $s_i$ to a discrete pole
% $z_i=e^{s_iT}$. Stable left-half-plane poles ($\mathrm{Re}\,s<0$) land
% *inside* the unit circle, which is the discrete-time stability region.
s_poles  = pole(G);
z_mapped = exp(s_poles*T);
fprintf('Continuous poles: %s\n', mat2str(s_poles.',4))
fprintf('Mapped e^{sT}:    %s\n', mat2str(z_mapped.',4))
fprintf('c2d poles:        %s\n', mat2str(pole(Gd).',4))

figure
pzmap(Gd)            % for a discrete model, pzmap draws the unit circle
title('Discrete Poles Inside the Unit Circle','Interpreter','latex','FontSize',15)
grid on

%% ZOH vs. Tustin (bilinear) discretization
% |c2d| supports several methods. *ZOH* matches the step response at the
% samples; *Tustin* applies the bilinear rule
% $s\to\frac{2}{T}\frac{z-1}{z+1}$, which better preserves the frequency
% response and never maps a stable pole outside the unit circle.
Gd_tustin = c2d(G, T, 'tustin');
figure
bode(G,'b', Gd,'r--', Gd_tustin,'g-.')
legend('Continuous','ZOH','Tustin','Interpreter','latex','FontSize',12)
title('Frequency Response: Discretization Methods','Interpreter','latex','FontSize',15)
grid on

%% How fast must we sample?
% A rule of thumb is to sample 20--30 times faster than the closed-loop
% bandwidth. Re-discretize at several sample times and overlay the step
% responses: as $T$ grows, the discrete model -- and any controller built
% from it -- drifts away from the continuous truth.
figure
hold on
step(G,'k')
for Ti = [0.2 0.5 1.0 2.0]
    step(c2d(G,Ti,'zoh'))
end
hold off
legend('Continuous','$T=0.2$','$T=0.5$','$T=1.0$','$T=2.0$', ...
    'Interpreter','latex','FontSize',11)
title('Coarser Sampling Degrades the Discrete Model','Interpreter','latex','FontSize',15)
ylabel('$y$','Interpreter','latex','FontSize',16)
set(get(gca,'YLabel'),'Rotation',0)
xlabel('$t$','Interpreter','latex','FontSize',16)

%% Summary
% * |c2d(G,T,method)| builds a discrete model: *ZOH* matches the sampled
%   step, *Tustin* matches the frequency response.
% * Continuous poles map to $z=e^{sT}$; the unit circle is the discrete
%   stability boundary.
% * Sample fast relative to the dynamics -- coarse sampling throws away
%   accuracy and, eventually, stability.
%
% *Next:* |DiscretePID.m| runs a controller in discrete time, and
% |DeadbeatControl.m| exploits sampling to settle in a finite number of
% steps.

%% Try it yourself
% * Halve the sample time |T| and notice the ZOH discrete step hug the
%   continuous curve more tightly.
% * Discretize with |'matched'| instead of |'zoh'| and compare the poles
%   against |exp(s_poles*T)|.
