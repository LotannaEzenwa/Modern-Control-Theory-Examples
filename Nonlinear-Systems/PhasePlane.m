%% Nonlinear Systems I: Phase-Plane Analysis
% *Understanding a second-order nonlinear system from its trajectories in
% the state plane.*
%
% Transfer functions and poles describe only *linear* systems. A
% second-order nonlinear system can still be understood graphically
% through its *phase portrait*: the family of trajectories drawn in the
% $(x_1,x_2)=(x,\dot{x})$ plane. In this tutorial you will:
%
% * draw the *vector field* and trajectories of a damped pendulum,
% * locate and classify its equilibria (stable focus vs. saddle), and
% * see a *limit cycle* in the Van der Pol oscillator -- a phenomenon with
%   no linear analog.
%
% Uses |ode45| (no toolbox required). Run with |publish('PhasePlane.m')|.

%% The pendulum as a state-space system
% $\ddot{\theta}+b\dot{\theta}+\sin\theta=0$ with $x_1=\theta$,
% $x_2=\dot{\theta}$ becomes
%
% $$ \dot{x}_1 = x_2, \qquad \dot{x}_2 = -b\,x_2 - \sin x_1. $$
b = 0.3;
pend = @(t,x) [x(2); -b*x(2) - sin(x(1))];

%% The vector field
% At every point the arrow $(\dot{x}_1,\dot{x}_2)$ shows which way a
% trajectory moves. Arrows are normalized to unit length so direction is
% readable everywhere.
[X1,X2] = meshgrid(linspace(-2*pi,2*pi,25), linspace(-4,4,21));
U = X2;
V = -b*X2 - sin(X1);
Nrm = hypot(U,V);  Nrm(Nrm==0) = 1;
figure
quiver(X1,X2, U./Nrm, V./Nrm, 0.5, 'Color',[.6 .6 .6])
hold on

%% Trajectories and equilibria
% Overlay trajectories from several initial conditions. Equilibria sit at
% $(\,k\pi,0\,)$: the *down* positions ($0,\pm2\pi$) are stable foci (the
% damping spirals trajectories in), while the *up* positions ($\pm\pi$)
% are saddles (unstable).
for x10 = -5:1.5:5
    for x20 = [-3 3]
        [~,xx] = ode45(pend,[0 25],[x10;x20]);
        plot(xx(:,1),xx(:,2),'b')
    end
end
plot([-2*pi 0 2*pi],[0 0 0],'go','MarkerFaceColor','g','MarkerSize',8)
plot([-pi pi],[0 0],'rx','MarkerSize',11,'LineWidth',2)
hold off
axis([-2*pi 2*pi -4 4])
legend('vector field','trajectories','stable (down)','saddle (up)', ...
    'Interpreter','latex','FontSize',11,'Location','northeastoutside')
title('Phase Portrait of the Damped Pendulum','Interpreter','latex','FontSize',15)
ylabel('$\dot{\theta}$','Interpreter','latex','FontSize',18); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$\theta$','Interpreter','latex','FontSize',18)

%% Classifying equilibria with the Jacobian
% Linearizing about an equilibrium $x^*$ gives the local behavior. The
% Jacobian is $J=[\,0\ \ 1\,;\ -\cos\theta\ \ -b\,]$.
J = @(th) [0 1; -cos(th) -b];
fprintf('Down (theta=0):  eigenvalues %s  -> stable focus\n', mat2str(eig(J(0)).',3))
fprintf('Up   (theta=pi): eigenvalues %s  -> saddle (unstable)\n', mat2str(eig(J(pi)).',3))

%% A limit cycle: the Van der Pol oscillator
% $\ddot{x}-\mu(1-x^2)\dot{x}+x=0$. For $\mu>0$ *every* trajectory --
% started inside or outside -- is drawn onto a single closed orbit, the
% *limit cycle*. A linear system can only spiral in or out; a sustained,
% amplitude-stable oscillation like this is intrinsically nonlinear.
mu  = 1;
vdp = @(t,x) [x(2); mu*(1-x(1)^2)*x(2) - x(1)];
figure
hold on
for r0 = [0.1 4]
    [~,xx] = ode45(vdp,[0 25],[r0;0]);
    plot(xx(:,1),xx(:,2),'LineWidth',1.2)
end
hold off
grid on; axis equal
legend('from inside ($x_0=0.1$)','from outside ($x_0=4$)', ...
    'Interpreter','latex','FontSize',11)
title('Van der Pol Limit Cycle ($\mu=1$)','Interpreter','latex','FontSize',15)
ylabel('$\dot{x}$','Interpreter','latex','FontSize',18); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$x$','Interpreter','latex','FontSize',18)

%% Summary
% * A phase portrait shows the global behavior of a 2nd-order nonlinear
%   system at a glance: where it settles, and where it cannot.
% * The Jacobian classifies each equilibrium locally (focus, node, saddle).
% * Nonlinear systems support behaviors -- multiple equilibria, limit
%   cycles -- that linear systems simply cannot.
%
% *Next:* |LyapunovStability.m| certifies stability *without* solving the
% equations.

%% Try it yourself
% * Raise the damping |b| and notice the pendulum trajectories spiral into
%   the down equilibrium faster (a tighter focus).
% * Set the Van der Pol |mu| to 3 and watch the limit cycle sharpen into a
%   relaxation oscillation.
