%% Block Diagrams and Mason's Gain Formula
% *Reducing an interconnection of blocks to one transfer function.*
%
% Ogata, _Modern Control Engineering_, Ch. 2.4--2.5.
%
% In this tutorial you will:
%
% * apply the series, parallel, and feedback reduction rules,
% * use |series|, |parallel|, and |feedback| to combine blocks, and
% * cross-check a multi-loop diagram with Mason's gain formula.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
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

%% Before vs. After Reduction: The Inner Loop vs. the Whole Diagram
% Block-diagram algebra collapses the multi-block diagram step by step.
% Compare the inner loop alone (before the outer feedback is closed) with
% the fully reduced system T_blocks (after) -- closing the outer loop
% speeds up and re-damps the response.
figure
step(series(inner,G3n),0:0.01:8)
hold on
step(T_blocks,0:0.01:8)
hold off
grid on
legend('Before (inner loop $\times G_3$, outer loop open)','After (fully reduced $T$)', ...
    'Interpreter','latex','FontSize',11)
title('Block Reduction: Before vs. After Closing the Outer Loop','Interpreter','latex','FontSize',15)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Try it yourself
% * Flip the inner loop to positive feedback with
%   |feedback(series(G1n,G2n),H1n,+1)| and notice the poles move the other way.
% * Set |H2s = 1| and re-derive |T_mason|, then check it against the numeric
%   |feedback| result -- they should still agree.
