%% Observability
% *Can we reconstruct the state from the output?*
%
% Ogata, _Modern Control Engineering_, Ch. 9.
%
% In this tutorial you will:
%
% * test observability with the |obsv| rank condition and the PBH test,
% * see the duality between observability and controllability, and
% * watch a second sensor restore a lost mode.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% A system is *completely observable* if the initial state $x(0)$ can be
% determined from knowledge of the output $y(t)$ over a finite time
% interval, given the input $u(t)$. For LTI systems this holds iff the
% *observability matrix*
%
% $$\mathcal{O} = \begin{bmatrix}C\\CA\\CA^2\\\vdots\\CA^{n-1}\end{bmatrix}$$
%
% has full rank $n$.

%% Example 1: An Observable System
A1 = [0 1; -2 -3];
C1 = [1 0];
Ob1 = obsv(A1,C1);
fprintf('Example 1: Observability matrix = \n')
disp(Ob1)
fprintf('rank = %d (n = %d) -> %s\n', rank(Ob1), size(A1,1), ...
    string(rank(Ob1)==size(A1,1)))

%% Example 2: An Unobservable System
% The sensor measures only a combination of states that is blind to one
% internal mode (a "hidden" state).
A2 = [-1 0; 0 -2];
C2 = [1 0];
Ob2 = obsv(A2,C2);
fprintf('\nExample 2: Observability matrix = \n')
disp(Ob2)
fprintf('rank = %d (n = %d) -> %s\n', rank(Ob2), size(A2,1), ...
    string(rank(Ob2)==size(A2,1)))
fprintf('State x2 has no effect on y: it is unobservable from this sensor.\n')

%% PBH Eigenvector Test for Observability
% $(A,C)$ is observable iff $\mathrm{rank}\begin{bmatrix}A-\lambda
% I\\C\end{bmatrix}=n$ for every eigenvalue of $A$.
eigsA2 = eig(A2);
for i = 1:length(eigsA2)
    lam = eigsA2(i);
    PBH = [A2-lam*eye(2); C2];
    fprintf('lambda=%.2f: rank[A-lambda*I; C] = %d\n', lam, rank(PBH))
end

%% Duality Between Controllability and Observability
% Kalman duality: $(A,C)$ is observable iff $(A^T,C^T)$ is controllable.
% This is exactly why observer gains are designed with `acker`/`place`
% applied to the transposed pair (see |StateObservers.m|).
Co_dual = ctrb(A1',C1');
Ob_direct = obsv(A1,C1);
fprintf('\nDuality check: ctrb(A^T,C^T)^T should equal obsv(A,C)\n')
fprintf('Match: %s\n', string(isequal(Co_dual', Ob_direct)))

%% Example 3: Adding a Second Sensor Restores Observability
% If a single output leaves a mode unobservable, adding measurements can
% recover full rank.
C2_aug = [1 0; 0 1];
Ob2_aug = obsv(A2,C2_aug);
fprintf('\nExample 2 with 2 outputs: rank(Ob) = %d (n=%d)\n', ...
    rank(Ob2_aug), size(A2,1))

%% Observability and State Reconstruction
% With $(A,C)$ observable, an observer can be built to asymptotically
% reconstruct $x(t)$ from $y(t)$ and $u(t)$ alone, even though $x(t)$
% itself is not directly measured -- the foundation of
% |StateObservers.m|.

%% Visualizing the Rank Condition
% Dually, the singular values of the observability matrix reveal the rank
% test: the unobservable system has a zero singular value, marking the
% hidden mode that no output combination exposes.
figure
bar([svd(Ob1), svd(Ob2)])
grid on
legend('Observable (Ex. 1)','Unobservable (Ex. 2)','Interpreter','latex','FontSize',12)
title('Singular Values of the Observability Matrix','Interpreter','latex','FontSize',18)
ylabel('$\sigma_i$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('Index $i$','Interpreter','latex','FontSize',20)
