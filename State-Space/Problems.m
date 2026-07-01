%% State-Space -- Worked Problems
% *Practice: controllability and observability of given systems.*
%
% Ogata, _Modern Control Engineering_, Ch. 9 (end-of-chapter style).
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.

%% Problem 1: Test Controllability and Observability
% For $\dot{x}=Ax+Bu,\ y=Cx$ with
%
% $$A=\begin{bmatrix}0&1&0\\0&0&1\\-6&-11&-6\end{bmatrix},\quad
%   B=\begin{bmatrix}0\\0\\1\end{bmatrix},\quad C=\begin{bmatrix}1&0&0\end{bmatrix}$$
%
% determine complete state controllability and observability.
A1 = [0 1 0; 0 0 1; -6 -11 -6];
B1 = [0;0;1];
C1 = [1 0 0];

Co1 = ctrb(A1,B1);
Ob1 = obsv(A1,C1);
fprintf('Problem 1: rank(Co) = %d, rank(Ob) = %d (n=%d)\n', ...
    rank(Co1), rank(Ob1), size(A1,1))
fprintf('Controllable: %d, Observable: %d\n', ...
    rank(Co1)==size(A1,1), rank(Ob1)==size(A1,1))

%% Problem 2: Find the Value of a Parameter that Destroys Controllability
% For
%
% $$A=\begin{bmatrix}-1&0\\0&-2\end{bmatrix},\quad
%   B=\begin{bmatrix}1\\\beta\end{bmatrix}$$
%
% find all $\beta$ for which the system is *not* controllable.
syms beta
A2 = [-1 0; 0 -2];
B2 = sym([1; beta]);
Co2 = [B2 A2*B2];
detCo2 = simplify(det(Co2));
fprintf('\nProblem 2: det(Co) = %s\n', char(detCo2))
beta_sol = solve(detCo2==0,beta);
fprintf('System is uncontrollable when beta = %s\n', char(beta_sol))

%% Problem 3: Observability of a Sensor Placement Choice
% For the same $A$ as Problem 2 with two candidate single-output
% matrices $C_a=[1\ 0]$ and $C_b=[1\ 1]$, determine which sensor
% placement(s) preserve observability.
A3 = [-1 0; 0 -2];
Ca = [1 0];
Cb = [1 1];
fprintf('\nProblem 3: rank(obsv(A,Ca)) = %d\n', rank(obsv(A3,Ca)))
fprintf('Problem 3: rank(obsv(A,Cb)) = %d\n', rank(obsv(A3,Cb)))
fprintf('Ca alone misses mode at s=-2 (decoupled, unmeasured); Cb observes both.\n')

%% Problem 4: Controllability/Observability of a Transfer Function Realization
% For $G(s)=\frac{s+2}{(s+1)(s+3)(s+4)}$, realize in controllable
% canonical form and verify controllability is automatic (by
% construction) while checking observability (which can fail if there is
% pole-zero cancellation).
G4 = tf([1 2],conv(conv([1 1],[1 3]),[1 4]));
sys4 = ss(G4);
A4 = sys4.A; B4 = sys4.B; C4 = sys4.C;
fprintf('\nProblem 4: rank(Co) = %d, rank(Ob) = %d (n=%d)\n', ...
    rank(ctrb(A4,B4)), rank(obsv(A4,C4)), size(A4,1))
fprintf('No pole-zero cancellation present, so both hold (n=%d, full rank).\n', size(A4,1))

%% Visualizing Problem 4: Pole-Zero Map
% The realization of $G(s)=\frac{s+2}{(s+1)(s+3)(s+4)}$ has its zero at
% $-2$ distinct from every pole, so no pole-zero cancellation occurs and
% the system is a minimal realization -- both controllable and observable.
figure
pzmap(sys4)
grid on
title('Problem 4: Pole-Zero Map (No Cancellation)','Interpreter','latex','FontSize',18)

%% Try it yourself
% * In Problem 2, change |A| and re-solve |detCo2==0| to find that system's
%   own uncontrollable |beta|.
% * In Problem 4, force a pole-zero cancellation (put the zero at -1) and
%   watch observability drop below full rank.
