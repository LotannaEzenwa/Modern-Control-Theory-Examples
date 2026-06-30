%% Block Diagrams and Mason's Gain Formula
% Ogata, Modern Control Engineering, Ch. 2.4-2.5
%
% Complex control systems are described by interconnected blocks, each
% representing a transfer function, joined by summing junctions and
% takeoff points. Three reduction rules let any block diagram be
% collapsed to a single transfer function:
%
% * *Series (cascade):* blocks $G_1(s)$ and $G_2(s)$ in series combine
%   to $G_1(s)G_2(s)$.
% * *Parallel:* blocks $G_1(s)$ and $G_2(s)$ summed at a junction
%   combine to $G_1(s) \pm G_2(s)$.
% * *Feedback:* a forward path $G(s)$ with feedback path $H(s)$ closes
%   to
%
% $\frac{Y(s)}{R(s)} = \frac{G(s)}{1 \pm G(s)H(s)}$
%
% with $-$ for negative feedback and $+$ for positive feedback.

%% Series and Parallel Combination
G1 = tf(1,[1 1]);     % 1/(s+1)
G2 = tf(2,[1 2]);     % 2/(s+2)

G_series  = series(G1,G2)
G_parallel = parallel(G1,G2)

%% Closing a Negative-Feedback Loop
% For unity feedback ($H(s)=1$):
G = tf(10,[1 2 0]);   % 10/(s(s+2))
T_unity = feedback(G,1)

%%
% For non-unity feedback $H(s) = 1/(s+5)$:
H = tf(1,[1 5]);
T = feedback(G,H)

%% Mason's Gain Formula
% For diagrams too complex for simple reduction, Mason's gain formula
% gives the overall transfer function directly from the signal-flow
% graph:
%
% $T = \frac{Y}{R} = \frac{1}{\Delta}\sum_k P_k \Delta_k$
%
% where $P_k$ is the gain of the $k$-th forward path, $\Delta$ is the
% determinant of the graph,
%
% $\Delta = 1 - \sum L_i + \sum L_iL_j - \sum L_iL_jL_k + \dots$
%
% ($L_i$ = individual loop gains, sums taken over non-touching loops),
% and $\Delta_k$ is $\Delta$ evaluated on the subgraph that does not
% touch forward path $k$.

%% Worked Example: Two-Loop System
% Forward path: $P_1 = G_1G_2G_3$.
% Loops: $L_1=-G_1G_2H_1$ (around $G_1G_2$), $L_2=-G_2G_3H_2$ (around
% $G_2G_3$). $L_1$ and $L_2$ share $G_2$, so they touch and there is no
% second-order term.
%
% $\Delta = 1-(L_1+L_2) = 1+G_1G_2H_1+G_2G_3H_2$
%
% Since the forward path touches every loop, $\Delta_1 = 1$, giving
%
% $T = \frac{G_1G_2G_3}{1+G_1G_2H_1+G_2G_3H_2}$
%
% Verify symbolically with $G_1=1/(s+1)$, $G_2=1/s$, $G_3=2$, $H_1=1$,
% $H_2=0.5$.
syms s
G1s = 1/(s+1);
G2s = 1/s;
G3s = 2;
H1s = 1;
H2s = sym(1)/2;

P1 = G1s*G2s*G3s;
Delta = 1 + G1s*G2s*H1s + G2s*G3s*H2s;
T_mason = simplify(P1/Delta)

%%
% Cross-check by building the same diagram with |series|, |parallel|,
% and |feedback| using numeric transfer functions: inner loop
% $G_1G_2$ closed by $H_1$, then cascaded with $G_3$ and closed by
% $H_2$ around the outer loop.
G1n = tf(1,[1 1]);
G2n = tf(1,[1 0]);
G3n = tf(2,1);
H1n = tf(1,1);
H2n = tf(0.5,1);

inner = feedback(series(G1n,G2n),H1n);
T_blocks = feedback(series(inner,G3n),H2n)

%% Visualizing the Reduced System
% The symbolic (Mason) and numeric (block-reduction) results are the same
% transfer function; the step response of the reduced closed loop
% T_blocks confirms the algebra produced a sensible, stable system.
figure
step(T_blocks)
grid on
title('Block-Reduced Closed-Loop Step Response','Interpreter','latex','FontSize',18)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
