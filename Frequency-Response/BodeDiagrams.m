%% Bode Diagrams
% Ogata, Modern Control Engineering, Ch. 7: Bode Diagram (Logarithmic
% Plot) Construction
%
% A Bode diagram plots $20\log_{10}|G(j\omega)|$ (dB) and
% $\angle G(j\omega)$ (deg) vs. $\log_{10}\omega$. Because $G(j\omega)$
% is normally a product of simple pole/zero/gain/integrator factors, and
% $\log|G_1G_2| = \log|G_1|+\log|G_2|$, the log-magnitude and phase of
% each factor can be sketched separately on a log-frequency axis and
% simply *added* -- this is the key advantage of the logarithmic plot.

%% Standard (Bode) Factors and Their Asymptotic Rules
%
% *1. Gain $K$*: contributes a constant $20\log_{10}|K|$ dB, $0^\circ$
% (or $180^\circ$ if $K<0$) phase, independent of $\omega$.
%
% *2. Integrator/differentiator $(j\omega)^{\pm1}$*: magnitude slope of
% $\mp20$ dB/decade through 0 dB at $\omega=1$; constant phase of
% $\mp90^\circ$.
%
% *3. First-order pole/zero $(1+j\omega T)^{\pm1}$*: low-frequency
% asymptote 0 dB (flat); high-frequency asymptote slope $\pm20$ dB/decade
% beyond the *corner frequency* $\omega=1/T$; phase goes from $0^\circ$
% to $\pm90^\circ$, passing through $\pm45^\circ$ at the corner.
%
% *4. Quadratic pole/zero
% $\left(1+2\zeta(j\omega/\omega_n)+(j\omega/\omega_n)^2\right)^{\pm1}$*:
% asymptotic slope $\pm40$ dB/decade beyond $\omega_n$; near the corner
% the actual curve deviates from the asymptote by an amount depending on
% $\zeta$ (resonant peak for small $\zeta$); phase goes from $0^\circ$ to
% $\mp180^\circ$.

%% Worked Example: Sketch vs. Exact Bode Plot
% $G(s) = \frac{10(s+2)}{s(s+10)}$. Rewrite in Bode (time-constant) form:
%
% $G(j\omega) = \frac{10\times2(1+j\omega/2)}{j\omega\times10(1+j\omega/10)}
%   = \frac{2(1+j\omega/2)}{j\omega(1+j\omega/10)}$
%
% Corner frequencies: zero at $\omega=2$, pole at $\omega=10$, plus an
% integrator (pole at the origin).
K_bode = 2;             % Bode gain after factoring to time-constant form
z1 = 2; p1 = 10;        % corner frequencies

w = logspace(-2,3,500);
s = 1j*w;

% Exact response
G = tf([10 20],[1 10 0]);
[mag,phase] = bode(G,w);
mag = squeeze(mag); phase = squeeze(phase);

% Asymptotic approximation, built term by term in dB/degrees
mag_dB_asym = 20*log10(K_bode) - 20*log10(w) ...
    + 20*log10(sqrt(1+(w/z1).^2)) - 20*log10(sqrt(1+(w/p1).^2));
% (the above already blends asymptote+exact for the 1st-order terms;
% a piecewise-linear version is shown for comparison)
mag_dB_piecewise = 20*log10(K_bode) - 20*log10(w);
mag_dB_piecewise = mag_dB_piecewise + 20*log10(max(w/z1,1)) - 20*log10(max(w/p1,1));

figure
semilogx(w,20*log10(mag),'b','LineWidth',1.5)
hold on
semilogx(w,mag_dB_piecewise,'r--')
hold off
legend('Exact','Asymptotic (piecewise-linear)','Interpreter','latex','FontSize',14)
title('Bode Magnitude: Asymptotic vs. Exact','Interpreter','latex','FontSize',20)
ylabel('dB','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$\omega$ (rad/s)','Interpreter','latex','FontSize',20)
grid on

%% Full Bode Diagram via the Control System Toolbox
figure
bode(G)
title('Bode Diagram: $G(s)=\frac{10(s+2)}{s(s+10)}$','Interpreter','latex','FontSize',20)
grid on

%% Gain Crossover and Phase Crossover Frequencies
% The *gain crossover frequency* $\omega_c$ is where $|G(j\omega)|=1$
% (0 dB); the *phase crossover frequency* $\omega_p$ is where
% $\angle G(j\omega)=-180^\circ$. These define the gain and phase
% margins (see |StabilityAnalysis.m|).
[Gm,Pm,Wcp,Wcg] = margin(G);
fprintf('Gain crossover frequency wc = %.4f rad/s\n', Wcg)
fprintf('Phase crossover frequency wp = %.4f rad/s\n', Wcp)
