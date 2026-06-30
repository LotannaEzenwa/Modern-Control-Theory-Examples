%% Introduction to the Root-Locus Method
% Ogata, Modern Control Engineering, Ch. 6: Root-Locus Analysis
%
% The root locus is the set of all locations in the $s$-plane that the
% closed-loop poles can occupy as a single real parameter (almost
% always the loop gain $K$) is varied from $0$ to $\infty$. For a
% unity-feedback system with open-loop transfer function $KG(s)$, the
% closed-loop characteristic equation is
%
% $1 + KG(s) = 0 \quad\Longleftrightarrow\quad G(s) = -\frac{1}{K}$

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
fprintf('Angle condition requires an odd multiple of 180 deg; here it is %s\n', ...
    char("not satisfied" + (mod(total_angle,360)==180 || mod(total_angle,360)==-180)*0 + ""))

% Find the actual gain at this approximate point (just for illustration)
K_at_s0 = abs(prod(s0-poles));
fprintf('If s0 were exactly on the locus, K = %.4f\n', K_at_s0)

%% Plotting the Locus Directly
% In practice, |rlocus| automates the angle/magnitude bookkeeping and
% sweeps $K$ over a default (or specified) range.
G = tf(1,poly(poles));
figure
rlocus(G)
title('Root Locus of $G(s)=\frac{1}{s(s+2)(s+4)}$','Interpreter','latex','FontSize',20)
grid on
