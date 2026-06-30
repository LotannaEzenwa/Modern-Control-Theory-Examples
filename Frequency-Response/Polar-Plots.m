%% Polar Plots
% Ogata, Modern Control Engineering, Ch. 7: Polar (Nyquist) Plots
%
% The polar plot of $G(j\omega)$ traces the locus of the complex number
% $G(j\omega) = \text{Re}\,G(j\omega) + j\,\text{Im}\,G(j\omega)$
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
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$\mathrm{Re}\,G(j\omega)$','Interpreter','latex','FontSize',20)

%% Worked Example: Type-1 System
% $G(s) = \frac{K}{s(Ts+1)}$, $K=1,T=1$. As $\omega\to0$,
% $G(j\omega)\to\infty\angle{-90^\circ}$ (the locus approaches the
% negative-imaginary axis asymptotically); as $\omega\to\infty$,
% $G(j\omega)\to0\angle{-180^\circ}$.
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
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$\mathrm{Re}\,G(j\omega)$','Interpreter','latex','FontSize',20)

%% Using the Control System Toolbox Directly
% |nyquist| automates the polar-plot construction (including negative
% frequencies, shown as the mirror image by symmetry).
figure
nyquist(G2)
title('Nyquist Plot via |nyquist|: $G(s)=\frac{1}{s(s+1)}$','Interpreter','latex','FontSize',20)
grid on
