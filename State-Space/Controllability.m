%% Controllability
% *Can the input steer the state anywhere we want?*
%
% Ogata, _Modern Control Engineering_, Ch. 9.
%
% In this tutorial you will:
%
% * test controllability with the |ctrb| rank condition,
% * use the PBH eigenvector test to pinpoint an uncontrollable mode, and
% * see why controllability is what makes pole placement possible.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% A system $\dot{x}=Ax+Bu$ is *completely state controllable* if, for any
% initial state $x(0)$ and any final state $x_1$, there exists an input
% $u(t)$ that transfers the system from $x(0)$ to $x_1$ in finite time.
% For LTI systems this is equivalent to the *controllability matrix*
%
% $$\mathbf{C} = [\,B\ \ AB\ \ A^2B\ \cdots\ A^{n-1}B\,]$$
%
% having full rank $n$ (Kalman's rank condition).

%% Example 1: A Controllable System
% Inverted-pendulum-like double integrator with cross-coupling:
A1 = [0 1; -2 -3];
B1 = [0; 1];
Co1 = ctrb(A1,B1);
fprintf('Example 1: Controllability matrix = \n')
disp(Co1)
fprintf('rank = %d (n = %d) -> %s\n', rank(Co1), size(A1,1), ...
    string(rank(Co1)==size(A1,1)))

%% Example 2: An Uncontrollable System
% Two decoupled subsystems where the input only reaches one of them:
A2 = [-1 0; 0 -2];
B2 = [1; 0];
Co2 = ctrb(A2,B2);
fprintf('\nExample 2: Controllability matrix = \n')
disp(Co2)
fprintf('rank = %d (n = %d) -> %s\n', rank(Co2), size(A2,1), ...
    string(rank(Co2)==size(A2,1)))
fprintf('State x2 cannot be influenced by u: it is uncontrollable.\n')

%% Geometric Interpretation: Reachable Subspace
% The columns of $\mathbf{C}$ span the *reachable subspace* -- the set
% of states attainable from the origin. For Example 2, this subspace is
% only the $x_1$-axis.
fprintf('\nReachable subspace basis (Example 2): column space of Co2\n')
disp(orth(Co2))

%% PBH (Popov-Belevitch-Hautus) Eigenvector Test
% An equivalent test: $(A,B)$ is controllable iff
% $\mathrm{rank}\,[\,A-\lambda I\ \ B\,]=n$ for every
% eigenvalue $\lambda$ of $A$. This identifies *which* mode is
% uncontrollable.
eigsA2 = eig(A2);
for i = 1:length(eigsA2)
    lam = eigsA2(i);
    PBH = [A2-lam*eye(2) B2];
    fprintf('lambda=%.2f: rank[A-lambda*I  B] = %d\n', lam, rank(PBH))
end

%% Controllability of a Three-State System (Mechanical Example)
% Two masses connected by a spring, force applied only to mass 1
% (Ogata-style coupled mechanical system), augmented with a damped third
% state to illustrate a 3x3 rank test.
A3 = [0 1 0; -2 -1 1; 0 1 -3];
B3 = [0;1;0];
Co3 = ctrb(A3,B3);
fprintf('\nExample 3: rank(Co3) = %d (n=%d)\n', rank(Co3), size(A3,1))

%% Effect of Controllability on Pole Placement
% A controllable pair permits arbitrary closed-loop pole placement via
% state feedback $u=-Kx$ (see |PolePlacement.m|); an uncontrollable mode
% cannot have its associated eigenvalue moved by any $K$.
K_test = place(A1,B1,[-3 -4]);
fprintf('\nFeedback gain K placing Example 1 poles at -3,-4: ')
disp(K_test)
fprintf('Closed-loop poles: ')
disp(eig(A1-B1*K_test)')

%% Visualizing the Rank Condition
% The singular values of the controllability matrix make the rank test
% visual: a controllable pair has all n singular values nonzero, while an
% uncontrollable pair has a zero singular value flagging the unreachable
% mode (its bar collapses to zero height below).
figure
bar([svd(Co1), svd(Co2)])
grid on
legend('Controllable (Ex. 1)','Uncontrollable (Ex. 2)','Interpreter','latex','FontSize',12)
title('Singular Values of the Controllability Matrix','Interpreter','latex','FontSize',18)
ylabel('$\sigma_i$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('Index $i$','Interpreter','latex','FontSize',20)

%% Try it yourself
% * Change |B2| to |[1;1]| and notice Example 2 become controllable -- the
%   input now reaches both modes.
% * Confirm you cannot |place| the eigenvalue of an uncontrollable mode no
%   matter what gain you choose.
