%% Introduction to Frequency-Response Analysis
% Ogata, Modern Control Engineering, Ch. 7: Frequency-Response Analysis
%
% The frequency response of a system is its steady-state response to a
% sinusoidal input. For a stable, linear time-invariant system with
% transfer function $G(s)$, if the input is
%
% $u(t) = A\sin(\omega t)$
%
% then the steady-state output is also sinusoidal, at the same
% frequency, but scaled in magnitude and shifted in phase:
%
% $y(t) = AM(\omega)\sin(\omega t + \phi(\omega))$
%
% where $M(\omega)=|G(j\omega)|$ and $\phi(\omega)=\angle G(j\omega)$.
% This follows directly from substituting $s=j\omega$ into the transfer
% function -- $G(j\omega)$ is called the *sinusoidal transfer function*.

%% Why $s=j\omega$
% The Laplace transform of $A\sin(\omega t)$ is
% $\frac{A\omega}{s^2+\omega^2}$, with poles at $s=\pm j\omega$ on the
% imaginary axis. For a stable $G(s)$, the partial-fraction expansion of
% $Y(s)=G(s)U(s)$ separates into terms from $G(s)$'s own poles (which
% decay to zero, the transient) and a term from the input poles at
% $\pm j\omega$, which persists forever as the steady-state sinusoidal
% response. Evaluating the residues at $s=\pm j\omega$ shows the
% steady-state output is fully characterized by $G(j\omega)$.

%% Worked Example
% $G(s) = \frac{1}{s+1}$. Evaluate $G(j\omega)$ at a few frequencies and
% confirm against |bode|/|freqresp|.
G = tf(1,[1 1]);
omegas = [0.1 1 2 10];
for w = omegas
    g = evalfr(G,1j*w);
    fprintf('w=%5.2f: |G(jw)|=%.4f, angle=%.2f deg\n', w, abs(g), angle(g)*180/pi)
end

%% Direct Verification via Simulation
% Drive $G(s)$ with $u(t)=\sin(2t)$ and confirm the steady-state output
% amplitude/phase match $|G(j2)|,\angle G(j2)$.
w_test = 2;
t = 0:0.01:30;
u = sin(w_test*t);
y = lsim(G,u,t);

g_test = evalfr(G,1j*w_test);
M = abs(g_test); phi = angle(g_test);
y_pred = M*sin(w_test*t + phi);

figure
hold on
plot(t,y,'b')
plot(t,y_pred,'r--')
hold off
legend('Simulated $y(t)$','Predicted $AM\sin(\omega t+\phi)$','Interpreter','latex','FontSize',14)
title('Frequency Response Verified by Simulation','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
xlim([20 30])

%% Standard Frequency-Response Plots
% Three equivalent graphical representations of $G(j\omega)$ are used
% throughout this chapter and the next:
%
% # *Bode diagram*: $20\log_{10}|G(j\omega)|$ (dB) and $\angle G(j\omega)$
%   (deg), each vs. $\log_{10}\omega$.
% # *Polar (Nyquist) plot*: $G(j\omega)$ traced in the complex plane as
%   $\omega$ sweeps $0\to\infty$.
% # *Log-magnitude-vs-phase (Nichols) plot*: magnitude (dB) vs. phase
%   (deg), parameterized by $\omega$.
figure
bode(G)
title('Bode Diagram of $G(s)=\frac{1}{s+1}$','Interpreter','latex','FontSize',20)
grid on
