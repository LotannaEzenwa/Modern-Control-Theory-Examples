%% Root-Locus Plots with MATLAB
% *Drawing a locus and reading gains and closed-loop poles off it.*
%
% Ogata, _Modern Control Engineering_, Ch. 6.
%
% In this tutorial you will:
%
% * plot a locus with |rlocus| and find closed-loop poles for given gains,
% * check the asymptotes and breakaway point by hand, and
% * find the gain at the imaginary-axis crossing (the stability limit).
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% This file demonstrates the |rlocus| command on a representative
% open-loop transfer function and shows how to read closed-loop pole
% locations and the corresponding gain directly from the plot.

%% Open-Loop System
% $G(s) = \frac{K}{s(s+1)(s+3)}$
num = 1;
den = conv([1 1 0],[1 3]);
G = tf(num,den);
poles_ol = pole(G)
zeros_ol = zero(G)

figure
rlocus(G)
title('Root Locus: $G(s)=\frac{1}{s(s+1)(s+3)}$','Interpreter','latex','FontSize',20)
grid on

%% Reading Gain and Closed-Loop Poles at a Chosen Point
% |rlocfind| (interactive) or, programmatically, |rlocus| with a
% specific gain vector can be used to find the closed-loop poles for a
% given $K$. Here we sweep a vector of gains and report the resulting
% pole locations directly.
Ks = [0.5 1 2 4 8];
for K = Ks
    cl_poles = pole(feedback(K*G,1));
    fprintf('K = %4.1f -> closed-loop poles: %s\n', K, mat2str(cl_poles,4))
end

%% Asymptotes and Breakaway Point (Hand Calculation Check)
% With three poles at $0,-1,-3$ and no finite zeros ($n=3,m=0$):
%
% $\sigma_a = \frac{(0)+(-1)+(-3)}{3} = -\frac{4}{3}$
%
% $\theta_a = 60^\circ, 180^\circ, 300^\circ$
sigma_a = sum(poles_ol)/numel(poles_ol);
fprintf('Asymptote centroid sigma_a = %.4f (expect -1.3333)\n', sigma_a)

% The breakaway point satisfies dK/ds = 0 for K = -1/G(s); solve
% numerically with the characteristic polynomial's derivative.
syms s
Gs = 1/(s*(s+1)*(s+3));
Ksym = -1/Gs;
dKds = diff(Ksym,s);
breakaway_candidates = double(solve(dKds==0,s));
disp('Breakaway candidates (real, between 0 and -1):')
disp(breakaway_candidates(imag(breakaway_candidates)==0))

%% Gain at the Imaginary-Axis Crossing (Stability Boundary)
% The locus crosses into the right half-plane at some gain $K_{cr}$;
% this is found from the Routh array of the closed-loop characteristic
% polynomial $s^3+4s^2+3s+K=0$ exactly as in
% |Transient and Steady-State/RouthStability.m|.
% Routh: s^3: [1 3], s^2: [4 K], s^1: [(12-K)/4], s^0: [K]
% Stability requires 0 < K < 12.
K_cr = 12;
p_cr = roots([1 4 3 K_cr]);
disp('Closed-loop poles at the stability boundary K=12:')
disp(p_cr)

%% Before vs. After: How the Poles Migrate with Gain
% The locus *is* the before/after story. At K=0 (before) the closed-loop
% poles sit exactly on the open-loop poles; as K grows they migrate along
% the branches, reaching the imaginary axis at K = Kcr (onset of
% instability). Overlay the K=0 starting points, an intermediate K, and
% the K=Kcr endpoints on the locus.
p0   = poles_ol;                  % K = 0   (before)
pmid = pole(feedback(6*G,1));     % K = 6   (intermediate)
% p_cr already holds the K = Kcr boundary poles
figure
rlocus(G)
hold on
plot(real(p0),imag(p0),'ks','MarkerSize',10,'LineWidth',1.5)
plot(real(pmid),imag(pmid),'bd','MarkerSize',8,'LineWidth',1.5)
plot(real(p_cr),imag(p_cr),'rx','MarkerSize',11,'LineWidth',1.5)
xline(0,'k:')
hold off
legend('Locus','$K=0$ (before)','$K=6$','$K=K_{cr}=12$ (boundary)', ...
    'Interpreter','latex','FontSize',11,'Location','southwest')
title('Closed-Loop Pole Migration: $K=0 \rightarrow K_{cr}$','Interpreter','latex','FontSize',18)
grid on   % keep rlocus's own "Real/Imaginary Axis" labels (it re-appends units,
          % which breaks a custom latex xlabel/ylabel)

%% Try it yourself
% * Sweep a finer gain vector and find where the branches cross the
%   imaginary axis -- that gain matches the Routh limit K=12.
% * Add a zero at -2 (|G = tf([1 2],den)|) and watch a branch terminate on
%   it instead of escaping to infinity.
