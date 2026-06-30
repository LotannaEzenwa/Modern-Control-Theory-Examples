%% Mechanical Systems
% *Newton's second law to transfer functions: mass-spring-damper systems.*
%
% Ogata, _Modern Control Engineering_, Ch. 3.
%
% In this tutorial you will:
%
% * derive the mass-spring-damper model from a free-body diagram,
% * see the translational/rotational analogy, and
% * build a two-mass system directly in state-space form.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% Mechanical systems obey Newton's second law, $\sum F = m\ddot{x}$.
% For a translational mass-spring-damper system with displacement
% $x(t)$ driven by force $u(t)$, the free-body diagram gives three
% opposing forces on the mass: the applied force $u$, a spring force
% $-kx$ (Hooke's law), and a damping force $-b\dot{x}$ (viscous
% friction, proportional to velocity). Summing forces:
%
% $m\ddot{x} = u - b\dot{x} - kx \quad\Rightarrow\quad
%   m\ddot{x} + b\dot{x} + kx = u$

%% Transfer Function of the Translational System
% Taking the Laplace transform with zero initial conditions:
%
% $(ms^2 + bs + k)X(s) = U(s) \quad\Rightarrow\quad
%   G(s) = \frac{X(s)}{U(s)} = \frac{1}{ms^2+bs+k}$
%
% With $m=1\,\mathrm{kg}$, $b=4\,\mathrm{N\cdot s/m}$,
% $k=20\,\mathrm{N/m}$:
m = 1; b = 4; k = 20;
G_trans = tf(1,[m b k])

figure
[xs,ts] = step(G_trans);
plot(ts, ones(size(ts)),'k--', ts, xs,'b','LineWidth',1.3)
legend('Applied force step (before)','Displacement $x(t)$ (after)','Interpreter','latex','FontSize',11,'Location','east')
title('Force In vs. Displacement Out (Mass-Spring-Damper)','Interpreter','latex','FontSize',15)
ylabel('amplitude','Interpreter','latex','FontSize',16)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Rotational Analog
% The rotational analog replaces force with torque $T$, mass with
% moment of inertia $J$, viscous friction coefficient $b$, and linear
% spring constant $k$ with torsional spring constant $k$. Newton's
% second law for rotation, $\sum T = J\ddot{\theta}$, gives
%
% $J\ddot{\theta} + b\dot{\theta} + k\theta = T$
%
% which is structurally identical to the translational equation under
% the analogy $x\leftrightarrow\theta$, $m\leftrightarrow J$,
% $u\leftrightarrow T$.
J = 0.5; b_rot = 1; k_rot = 4;
G_rot = tf(1,[J b_rot k_rot])

%% Two-Mass System
% A more complex example: two masses $m_1,m_2$ connected by a spring
% $k$ and damper $b$ between them, with force $u$ applied to $m_1$ and
% $m_2$ free to move. Free-body diagrams for each mass give
%
% $m_1\ddot{x}_1 = u - k(x_1-x_2) - b(\dot{x}_1-\dot{x}_2)$
%
% $m_2\ddot{x}_2 = k(x_1-x_2) + b(\dot{x}_1-\dot{x}_2)$
%
% In state-space form with state $z=[x_1\ \dot{x}_1\ x_2\ \dot{x}_2]^T$:
m1 = 1; m2 = 1; k2 = 10; b2 = 2;
A = [0 1 0 0
     -k2/m1 -b2/m1 k2/m1 b2/m1
     0 0 0 1
     k2/m2 b2/m2 -k2/m2 -b2/m2];
B = [0; 1/m1; 0; 0];
C = [1 0 0 0];
D = 0;
sys_2mass = ss(A,B,C,D);

figure
step(sys_2mass)
title('Two-Mass System Step Response ($x_1$)','Interpreter','latex','FontSize',20)
ylabel('$x_1(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
