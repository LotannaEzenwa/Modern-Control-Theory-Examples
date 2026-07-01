%% Polar Plots
% *Tracing $G(j\omega)$ in the complex plane.*
%
% Ogata, _Modern Control Engineering_, Ch. 7.
%
% In this tutorial you will:
%
% * plot the polar locus of a first-order and a type-1 system by hand,
% * verify it against |nyquist|, and
% * see how loop gain scales the locus toward the critical point $-1$.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% The polar plot of $G(j\omega)$ traces the locus of the complex number
% $G(j\omega) = \mathrm{Re}\,G(j\omega) + j\,\mathrm{Im}\,G(j\omega)$
% in the complex plane as $\omega$ sweeps from $0$ to $\infty$. Unlike
% the Bode diagram, magnitude and phase are shown on a single plot, at
% the cost of frequency no longer being an explicit axis (it is only a
% parameter along the curve).

%% Worked Example: First-Order System
% $G(s) = \frac{1}{1+j\omega T}$, $T=1$.
%
% $G(j\omega) = \frac{1}{1+\omega^2T^2} - j\frac{\omega T}{1+\omega^2T^2}$
%
% At $\omega=0$: $G=1+j0$. As $\omega\to\infty$: $G\to0-j0$. This traces
% a semicircle in the lower half-plane, centered at $(0.5,0)$ with radius
% $0.5$ -- a classical result, verified below algebraically.
T = 1;
w = logspace(-2,3,1000);
Gjw = 1./(1+1j*w*T);
re = real(Gjw); im = imag(Gjw);

% Verify it lies on the circle (re-0.5)^2 + im^2 = 0.25
resid = (re-0.5).^2 + im.^2 - 0.25;
fprintf('Max deviation from ideal semicircle: %.2e\n', max(abs(resid)))

figure
plot(re,im,'b','LineWidth',1.5)
hold on
plot(re(1),im(1),'go','MarkerSize',8,'MarkerFaceColor','g')
plot(0,0,'k+')
hold off
axis equal
grid on
title('Polar Plot: $G(j\omega)=\frac{1}{1+j\omega}$','Interpreter','latex','FontSize',20)
ylabel('$\mathrm{Im}\,G(j\omega)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$\mathrm{Re}\,G(j\omega)$','Interpreter','latex','FontSize',20)

%% Worked Example: Type-1 System
% $G(s) = \frac{K}{s(Ts+1)}$, $K=1,T=1$. As $\omega\to0$,
% $G(j\omega)\to\infty\angle{-90^\circ}$ (the locus runs off to
% $-j\infty$ along the vertical asymptote $\mathrm{Re}=-KT=-1$); as
% $\omega\to\infty$, $G(j\omega)\to0\angle{-180^\circ}$.
G2 = tf(1,[1 1 0]);
w2 = logspace(-2,2,500);
Gjw2 = squeeze(freqresp(G2,w2));

figure
plot(real(Gjw2),imag(Gjw2),'b','LineWidth',1.5)
hold on
plot(0,0,'k+')
hold off
grid on
xlim([-3 1]); ylim([-5 1])
title('Polar Plot: $G(s)=\frac{1}{s(s+1)}$ (Type-1)','Interpreter','latex','FontSize',20)
ylabel('$\mathrm{Im}\,G(j\omega)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$\mathrm{Re}\,G(j\omega)$','Interpreter','latex','FontSize',20)

%% Using the Control System Toolbox Directly
% |nyquist| automates the polar-plot construction (including negative
% frequencies, shown as the mirror image by symmetry).
figure
nyquist(G2)
title('Nyquist Plot via |nyquist|: $G(s)=\frac{1}{s(s+1)}$','Interpreter','latex','FontSize',20)
grid on

%% Before vs. After: Gain Scales the Polar Plot Toward -1
% Raising the loop gain scales the whole polar plot radially while the -1
% point (red) stays fixed, so a higher gain pushes the locus closer to
% (and eventually around) -1 -- the geometric onset of instability that
% the Nyquist criterion formalizes.
G2a = tf(1,[1 1 0]); G2b = tf(4,[1 1 0]);
figure
hold on
plot(real(squeeze(freqresp(G2a,w2))),imag(squeeze(freqresp(G2a,w2))),'b','LineWidth',1.3)
plot(real(squeeze(freqresp(G2b,w2))),imag(squeeze(freqresp(G2b,w2))),'r','LineWidth',1.3)
plot(-1,0,'rp','MarkerSize',13,'MarkerFaceColor','r')
hold off
grid on
xlim([-4 1]); ylim([-6 1])
legend('Before ($K=1$)','After ($K=4$)','$-1$ point','Interpreter','latex','FontSize',12)
title('Polar Plot: Gain Pushes the Locus Toward $-1$','Interpreter','latex','FontSize',15)
ylabel('$\mathrm{Im}$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$\mathrm{Re}$','Interpreter','latex','FontSize',20)

%% Try it yourself
% * Raise the type-1 gain from 4 toward 8 and notice the locus swing closer
%   to the -1 point -- the geometric approach to instability.
% * Increase |T| in the first example and watch the semicircle radius stay
%   0.5 but the frequencies redistribute along it.
