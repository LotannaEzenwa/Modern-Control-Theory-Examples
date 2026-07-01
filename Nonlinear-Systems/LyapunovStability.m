%% Nonlinear Systems II: Lyapunov Stability
% *Proving an equilibrium is stable without ever solving the equations.*
%
% Lyapunov's *direct method* certifies stability by finding an
% energy-like function $V(x)>0$ that *decreases* along every trajectory
% ($\dot{V}\le 0$). If such a $V$ exists, the equilibrium is stable -- no
% closed-form solution required. You will:
%
% * use the pendulum's physical energy as a Lyapunov function and watch it
%   decay along a simulated trajectory, and
% * for a linear system, solve the *Lyapunov equation*
%   $A^TP+PA=-Q$ with |lyap| and use $V=x^TPx$.
%
% Run with |publish('LyapunovStability.m')|, or step through with *Ctrl+Enter*.

%% Energy as a Lyapunov function (damped pendulum)
% For $\ddot{\theta}+b\dot{\theta}+\sin\theta=0$, the total energy
%
% $$ V(\theta,\dot{\theta}) = \tfrac{1}{2}\dot{\theta}^2 + (1-\cos\theta) \ge 0 $$
%
% is a natural candidate. Differentiating along trajectories gives
% $\dot{V}=-b\,\dot{\theta}^2\le 0$: energy can only dissipate, so the
% hanging-down equilibrium is stable.
b = 0.4;
pend = @(t,x)[x(2); -b*x(2) - sin(x(1))];
[t,x] = ode45(pend,[0 25],[2.5; 0]);
V = 0.5*x(:,2).^2 + (1 - cos(x(:,1)));

figure
subplot(2,1,1)
plot(t, x(:,1),'b','LineWidth',1.2)
grid on
title('Pendulum Angle Settling to the Stable Equilibrium','Interpreter','latex','FontSize',14)
ylabel('$\theta$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
subplot(2,1,2)
plot(t, V,'r','LineWidth',1.2)
grid on
title('Lyapunov Function $V$ Decreases Monotonically','Interpreter','latex','FontSize',14)
ylabel('$V$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$t$','Interpreter','latex','FontSize',16)

fprintf('V(0) = %.4f, V(end) = %.4f (energy dissipated to ~0)\n', V(1), V(end))

%% The Lyapunov equation for a linear system
% For $\dot{x}=Ax$, a classical theorem says: the origin is asymptotically
% stable *iff*, for any chosen $Q=Q^T>0$, the equation
%
% $$ A^TP + PA = -Q $$
%
% has a unique solution $P=P^T>0$. Then $V=x^TPx$ is a Lyapunov function.
% MATLAB's |lyap(A',Q)| solves exactly this.
A = [0 1; -2 -3];
Q = eye(2);
P = lyap(A', Q);
fprintf('\nP =\n'); disp(P)
fprintf('P positive definite?  %d  (eigenvalues %s)\n', all(eig(P)>0), mat2str(eig(P).',3))
fprintf('Residual ||A''P + PA + Q|| = %.2e\n', norm(A'*P + P*A + Q))

%% Visualizing V = x'Px as a bowl the trajectory rolls down
% The level sets of $V=x^TPx$ are nested ellipses. A trajectory of
% $\dot{x}=Ax$ crosses them strictly inward, because $\dot{V}=-x^TQx<0$.
[t2,x2] = ode45(@(t,x) A*x, [0 8], [4; 0]);
Vlin = sum((x2*P).*x2, 2);            % V(t) = x(t)' P x(t)

[g1,g2] = meshgrid(linspace(-5,5,60), linspace(-8,8,60));
Vgrid = P(1,1)*g1.^2 + 2*P(1,2)*g1.*g2 + P(2,2)*g2.^2;
figure
contour(g1,g2,Vgrid, 12, 'Color',[.7 .7 .7])
hold on
plot(x2(:,1), x2(:,2),'b','LineWidth',1.5)
plot(0,0,'k+','MarkerSize',10,'LineWidth',1.5)
hold off
axis equal
title('Trajectory Crossing the Level Sets of $V=x^TPx$ Inward','Interpreter','latex','FontSize',14)
ylabel('$x_2$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$x_1$','Interpreter','latex','FontSize',16)

%% Summary
% * Lyapunov's direct method proves stability from an energy function that
%   decreases -- no solution of the ODE needed.
% * Physical energy is often a ready-made Lyapunov function.
% * For linear systems the search is automatic: solve $A^TP+PA=-Q$ with
%   |lyap| and check $P>0$.

%% Try it yourself
% * Start the pendulum near the top (|[3.0;0]|) and notice V still decreases
%   monotonically -- energy dissipates from any initial swing.
% * Change |Q| to |diag([10 1])| and confirm |lyap| returns a different
%   P>0 with the same near-zero residual.
