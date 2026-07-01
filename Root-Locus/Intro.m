%% Introduction to the Root-Locus Method
% *Where the closed-loop poles go as the loop gain varies from 0 to infinity.*
%
% Ogata, _Modern Control Engineering_, Ch. 6.
%
% In this tutorial you will:
%
% * read the *angle* and *magnitude* conditions that define the locus,
% * learn the construction rules (branches, asymptotes, breakaway points), and
% * draw and read a locus with |rlocus|.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% The root locus is the set of all locations in the $s$-plane that the
% closed-loop poles can occupy as a single real parameter (almost
% always the loop gain $K$) is varied from $0$ to $\infty$. For a
% unity-feedback system with open-loop transfer function $KG(s)$, the
% closed-loop characteristic equation is
%
% $1 + KG(s) = 0 \quad\Leftrightarrow\quad G(s) = -\frac{1}{K}$

%% The Angle and Magnitude Conditions
% Writing $G(s)$ in pole-zero form,
%
% $G(s) = \frac{\prod_{j=1}^{m}(s-z_j)}{\prod_{i=1}^{n}(s-p_i)}$
%
% a point $s_0$ lies on the root locus for some $K>0$ if and only if it
% satisfies the *angle condition*
%
% $\sum_{j=1}^{m}\angle(s_0-z_j) - \sum_{i=1}^{n}\angle(s_0-p_i)
%   = \pm180^\circ(2k+1), \quad k=0,1,2,\ldots$
%
% The corresponding gain at that point is then found from the
% *magnitude condition*
%
% $K = \frac{\prod_{i=1}^{n}|s_0-p_i|}{\prod_{j=1}^{m}|s_0-z_j|}$
%
% The angle condition alone determines the *shape* of the locus
% (independent of $K$); the magnitude condition is then used to read
% off the gain at any particular point on that shape.

%% Basic Construction Rules (Summary)
% For $KG(s)H(s)$ with $n$ poles and $m$ zeros ($n \ge m$):
%
% # The locus has $n$ branches, starting ($K=0$) at the open-loop poles
%   and ending ($K=\infty$) at the open-loop zeros or at infinity.
% # The locus is symmetric about the real axis.
% # A point on the real axis is part of the locus if the total number
%   of real poles and zeros to its right is odd.
% # $n-m$ branches go to infinity along asymptotes centered at
%   $\sigma_a = \frac{\sum p_i - \sum z_j}{n-m}$ with angles
%   $\theta_a = \frac{180^\circ(2k+1)}{n-m}$.
% # Breakaway/break-in points on the real axis occur where
%   $\frac{dK}{ds}=0$.

%% Worked Verification of the Angle Condition
% Consider $G(s) = \frac{1}{s(s+2)(s+4)}$ and test whether
% $s_0=-1+j1.5$ lies (approximately) on the locus.
poles = [0 -2 -4];
s0 = -1 + 1.5j;

angles = angle(s0 - poles) * 180/pi;
total_angle = sum(angles);
fprintf('Pole angles at s0: %s degrees\n', mat2str(angles,4))
fprintf('Sum of pole angles = %.2f degrees\n', total_angle)

% With no finite zeros, the open-loop phase is angle(G(s0)) = -sum(pole
% angles). The angle condition is satisfied (s0 lies on the locus) when
% this phase is an odd multiple of 180 deg, i.e. when the residual below
% is zero.
ol_phase = -total_angle;
resid = mod(ol_phase, 360) - 180;     % 0 == exactly on the locus
fprintf('Open-loop phase at s0 = %.2f deg\n', ol_phase)
fprintf('Deviation from the angle condition = %.2f deg\n', resid)
if abs(resid) < 1
    fprintf('s0 satisfies the angle condition (it lies on the locus).\n')
else
    fprintf('s0 is off the locus by %.2f deg (this test point is only approximate).\n', resid)
end

% Find the gain that would place a closed-loop pole at s0 (magnitude
% condition); only exactly valid once s0 is on the locus.
K_at_s0 = abs(prod(s0-poles));
fprintf('Magnitude-condition gain at s0: K = %.4f\n', K_at_s0)

%% Plotting the Locus Directly
% In practice, |rlocus| automates the angle/magnitude bookkeeping and
% sweeps $K$ over a default (or specified) range.
G = tf(1,poly(poles));
figure
rlocus(G)
title('Root Locus of $G(s)=\frac{1}{s(s+2)(s+4)}$','Interpreter','latex','FontSize',20)
grid on

%% Before vs. After: Starting Points and Where They Go
% At K=0 the closed-loop poles coincide with the open-loop poles (the
% "before"); as K increases they trace the locus. Overlay the K=0 poles,
% the test point s0, and the closed-loop poles for a sample K so the
% migration the locus encodes is explicit.
K_demo = 20;
cl_demo = pole(feedback(K_demo*G,1));
figure
rlocus(G)
hold on
plot(real(poles),imag(poles),'ks','MarkerSize',10,'LineWidth',1.5)
plot([real(s0) real(s0)],[imag(s0) -imag(s0)],'rp','MarkerSize',13,'MarkerFaceColor','r')
plot(real(cl_demo),imag(cl_demo),'bd','MarkerSize',8,'LineWidth',1.5)
hold off
legend('Locus','$K=0$ poles (before)','Test point $s_0$','$K=20$ poles (after)', ...
    'Interpreter','latex','FontSize',11)
title('From Open-Loop Poles ($K=0$) Along the Locus','Interpreter','latex','FontSize',17)
grid on   % keep rlocus's own axis labels (it re-appends units, which breaks
          % a custom latex xlabel/ylabel)

%% Try it yourself
% * Move the test point to |s0 = -1 + 2j| and see whether the angle-
%   condition residual gets closer to zero (nearer the true locus).
% * Add a zero (|G = tf([1 3],poly(poles))|) and notice the branches bend
%   toward it instead of all running to infinity.
