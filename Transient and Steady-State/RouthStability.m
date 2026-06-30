%% Routh-Hurwitz Stability Criterion
% Ogata, Modern Control Engineering, Ch. 5: Routh's Stability Criterion
%
% A linear system is stable if and only if all poles of its closed-loop
% transfer function lie in the open left half of the $s$-plane. The
% Routh-Hurwitz criterion determines how many roots of a polynomial
%
% $a_0s^n + a_1s^{n-1} + \cdots + a_{n-1}s + a_n = 0$
%
% lie in the right half-plane *without solving for the roots
% explicitly* -- useful both as a hand-calculation tool and, as shown
% below, for finding a symbolic range of a design parameter (e.g. a
% controller gain) for which a closed-loop system remains stable.

%% Necessary Condition
% A necessary (but not sufficient) condition for stability is that all
% coefficients $a_0,\ldots,a_n$ be present and of the same sign. If any
% coefficient is zero or negative (with others positive), the system
% is immediately known to be unstable -- no further test is needed.

%% Routh Array Construction
% For $a_0s^4+a_1s^3+a_2s^2+a_3s+a_4=0$, the array is
%
%   s^4 |  a0   a2   a4
%   s^3 |  a1   a3   0
%   s^2 |  b1   b2
%   s^1 |  c1
%   s^0 |  d1
%
% with
%
% $b_1 = \frac{a_1a_2-a_0a_3}{a_1}, \quad
%   b_2 = \frac{a_1a_4-a_0\cdot0}{a_1}=a_4$
%
% $c_1 = \frac{b_1a_3-a_1b_2}{b_1}$,
% $\quad d_1 = \frac{c_1b_2-b_1\cdot0}{c_1}=b_2$
%
% The number of sign changes in the first column equals the number of
% roots in the right half-plane; the system is stable iff there are no
% sign changes (all first-column entries strictly positive, given
% $a_0>0$).

%% Worked Example: Numeric Routh Array
% $s^4+2s^3+3s^2+4s+5=0$
a = [1 2 3 4 5];
a0=a(1); a1=a(2); a2=a(3); a3=a(4); a4=a(5);

b1 = (a1*a2-a0*a3)/a1;
b2 = a4;
c1 = (b1*a3-a1*b2)/b1;
d1 = b2;

first_col = [a0 a1 b1 c1 d1];
fprintf('Routh first column: %s\n', mat2str(first_col,4))
sign_changes = sum(diff(sign(first_col)) ~= 0);
fprintf('Sign changes = %d -> %d right-half-plane root(s)\n', sign_changes, sign_changes)

% Verify against the actual roots
r = roots(a);
disp('Roots:'); disp(r)
disp('Number with positive real part:'); disp(sum(real(r)>0))

%% Worked Example: Stability Range for a Gain Parameter
% Consider a unity-feedback system with forward path
%
% $G(s) = \frac{K}{s(s+1)(s+2)}$
%
% The closed-loop characteristic equation is
%
% $1+G(s) = 0 \;\Rightarrow\; s(s+1)(s+2)+K = 0
%   \;\Rightarrow\; s^3+3s^2+2s+K = 0$
%
% Routh array:
%
%   s^3 |  1    2
%   s^2 |  3    K
%   s^1 |  (6-K)/3
%   s^0 |  K
%
% Stability requires every first-column entry positive:
% $3>0$ (always), $(6-K)/3>0 \Rightarrow K<6$, and $K>0$. Hence
%
% $0 < K < 6$
syms s K
charpoly = s*(s+1)*(s+2) + K;
charpoly = expand(charpoly)

% Numerically sweep K and confirm the closed-loop poles cross into the
% right half-plane exactly at the predicted boundary K = 6.
Ks = [0.1 1 3 5.9 6 6.1 8];
for k = Ks
    p = roots([1 3 2 k]);
    fprintf('K = %5.2f -> max real part of poles = %7.4f\n', k, max(real(p)))
end

%% Special Case: Zero in the First Column
% If a first-column entry is zero (but the row is not entirely zero),
% replace it with a small positive constant $\epsilon$ and continue;
% the sign of the limit as $\epsilon\to0^+$ reveals the sign change.
% If an entire row is zero, it indicates roots symmetric about the
% origin (e.g. purely imaginary pairs), and the array is continued
% using the derivative of the auxiliary polynomial formed from the row
% above. These special cases are not required for the worked examples
% above, which both have well-defined, nonzero first columns.

%% Visualizing the Stability Boundary
% Sweeping the gain K of the $K/(s(s+1)(s+2))$ example and plotting the
% maximum real part of the closed-loop poles shows them crossing into the
% right half-plane exactly at K = 6 -- the boundary the Routh array
% predicted analytically above.
K_sweep = 0:0.05:12;
max_re = zeros(size(K_sweep));
for ii = 1:numel(K_sweep)
    max_re(ii) = max(real(roots([1 3 2 K_sweep(ii)])));
end
figure
plot(K_sweep, max_re, 'b', 'LineWidth', 1.5)
hold on
yline(0,'k--')
xline(6,'r:','K = 6')
hold off
grid on
title('Stability Boundary: Max Pole Real Part vs. Gain','Interpreter','latex','FontSize',18)
ylabel('$\max_i\,\mathrm{Re}(s_i)$','Interpreter','latex','FontSize',16)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$K$','Interpreter','latex','FontSize',20)
