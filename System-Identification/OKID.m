%% System Identification II: Observer/Kalman Filter Identification (OKID)
% *Identifying lightly damped systems -- and shrugging off the unknown
% initial state -- with an embedded observer.*
%
% The plain impulse-response fit of |Intro.m| needs about one Markov
% parameter per sample of settling time, which is hopeless for lightly
% damped or oscillatory systems. *OKID* (Juang) sidesteps this: it
% identifies the Markov parameters of an *observer* form whose response
% dies out in a handful of steps (a deadbeat observer), then algebraically
% recovers the true system Markov parameters.
%
% This tutorial drives the repository's own root-level helpers --
% |YV_Form_nonzero|, |pinv2|, and |recover_SYSMP| -- the same tools behind
% the |Final/| exam problems. You will:
%
% * form the observer regression and solve it with a robust pseudoinverse,
% * recover the *observer* then the *system* Markov parameters, and
% * confirm they match the true impulse response, even from a nonzero
%   initial state.
%
% Run with |publish('OKID.m')|, or step through with *Ctrl+Enter*.

%% A known (undamped) system
% An undamped oscillator sampled at $T=0.2$ s -- the kind of ringing system
% a plain FIR fit chokes on, because its impulse response never decays.
A_c = [0 1; -1 0];
B_c = [0; 1];
C   = [1 0];
D   = 0;
T   = 0.2;
sysd = c2d(ss(A_c,B_c,C,D), T);
A = sysd.A; B = sysd.B; Cd = sysd.C; Dd = sysd.D;
n = size(A,1);
q = size(Cd,1);        % outputs
m = size(B,2);         % inputs

%% Run an experiment (with a nonzero initial state)
% Random input, and -- to make the point -- a nonzero initial condition the
% identifier is never told about.
rng(1);
ll = 60;
u  = randn(1,ll);
x0 = [1; 0];
Y  = lsim(sysd, u(:), (0:ll-1)*T, x0).';     % Y is q-by-ll

%% The deadbeat observer at the heart of OKID
% Pick an observer gain $G$ that places the eigenvalues of $A+GC$ at the
% origin. Then $\bar{A}=A+GC$ is *nilpotent*, so only $p=n$ observer Markov
% parameters are nonzero and the identification is finite.
G = acker(A', -Cd', zeros(1,n)).';
p = n;
fprintf('eig(A+GC) = %s   (deadbeat: all ~0)\n', mat2str(eig(A+G*Cd).',3))

%% Solve the observer regression with the repo helpers
% |YV_Form_nonzero| stacks the input/output record into the regression
% matrices $\bar{Y},\bar{V}$; |pinv2| is a tolerance-truncated SVD
% pseudoinverse that keeps the solve well conditioned. Their product is the
% row of observer Markov parameters.
[Y_bar, V_bar] = YV_Form_nonzero(u, Y, p);
cap_y_hat = Y_bar.' * pinv2(V_bar, 1e-5);

%% Recover the system Markov parameters
% |recover_SYSMP| runs the OKID recursion that converts the *observer*
% Markov parameters into the *system* Markov parameters $Y_k=CA^{k-1}B$.
n_mp = 20;
RSMP = recover_SYSMP(cap_y_hat, n_mp, q, m);

SMP = zeros(q, n_mp);            % true values for comparison
SMP(:,1) = Dd;
for i = 2:n_mp
    SMP(:,i) = Cd*A^(i-2)*B;
end

figure
stem(0:n_mp-1, SMP, 'b', 'filled')
hold on
stem(0:n_mp-1, RSMP, 'r')
hold off
grid on
legend('True $Y_k$','Recovered via OKID','Interpreter','latex','FontSize',13)
title('System Markov Parameters Recovered by OKID','Interpreter','latex','FontSize',15)
ylabel('$Y_k$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$k$','Interpreter','latex','FontSize',16)

fprintf('Relative error, recovered vs. true Markov parameters: %.2e\n', ...
    norm(RSMP - SMP)/norm(SMP))

%% Why it worked from a nonzero initial state
% *Notice that* we never handed the identifier $x_0$. The "_nonzero" in
% |YV_Form_nonzero| refers to exactly this: the regression starts only
% after the deadbeat observer has forgotten the initial condition
% ($k\ge p$), so the unknown $x_0$ drops out of the equations.

%% Try it yourself
% * Add measurement noise -- change the output line to
%   |Y = lsim(...).' + 0.005*randn(q,ll);| -- and watch |pinv2|'s tolerance
%   keep the recovery stable (try loosening it to |1e-2|).
% * Increase the record length |ll| and see the recovery error shrink.
% * Swap in a lightly damped plant, |c2d(ss(tf(1,[1 0.2 1])),T)|, and
%   confirm OKID still needs only |p = n| observer parameters.
%
% *Next:* |ERA.m| turns these Markov parameters into a state-space model
% $(A,B,C)$.

%% Summary
% * OKID identifies the Markov parameters of a deadbeat *observer* form,
%   which are finite in number even for undamped systems.
% * The root helpers |YV_Form_nonzero| / |pinv2| / |recover_SYSMP| do the
%   regression and the recursion back to system Markov parameters.
% * Starting the regression after the observer settles makes the method
%   immune to the unknown initial state.
