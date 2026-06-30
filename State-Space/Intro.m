%% Introduction to State-Space Representation
% Ogata, Modern Control Engineering, Ch. 9: State-Space Representation of
% Dynamic Systems
%
% A state-space model describes a system by a set of first-order
% differential equations in the *state vector* $x(t)\in\mathbb{R}^n$,
% together with an algebraic output equation:
%
% $$\dot{x}(t) = Ax(t)+Bu(t)$$
%
% $$y(t) = Cx(t)+Du(t)$$
%
% where $A$ ($n\times n$) is the *system matrix*, $B$ ($n\times r$) the
% *input matrix*, $C$ ($m\times n$) the *output matrix*, and $D$
% ($m\times r$) the *direct transmission matrix*. The state $x(t)$
% summarizes all information about the past needed (with future inputs)
% to determine the future response -- unlike the transfer function, the
% state-space form is not restricted to single-input single-output (SISO)
% zero-initial-condition systems.

%% Example: Mass-Spring-Damper in State-Space Form
% $m\ddot{y}+b\dot{y}+ky=u$ with $m=1,\ b=2,\ k=5$. Choosing
% $x_1=y,\ x_2=\dot{y}$:
%
% $$\dot{x}_1 = x_2$$
%
% $$\dot{x}_2 = -\frac{k}{m}x_1-\frac{b}{m}x_2+\frac{1}{m}u$$
m = 1; b = 2; k = 5;
A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;
sys_ss = ss(A,B,C,D);

%%
% The `ss` object can be queried for its matrices directly, simulated
% with `step`/`impulse`/`lsim`, and converted to other model forms.
figure
step(sys_ss)
title('State-Space Model: Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% State-Space to Transfer Function
% MATLAB computes $G(s)=C(sI-A)^{-1}B+D$ via `tf(sys_ss)`.
G_from_ss = tf(sys_ss);
fprintf('Transfer function recovered from state-space:\n')
G_from_ss

%%
% Direct verification: build the same plant from physical parameters as
% a transfer function and confirm it matches.
G_direct = tf(1,[m b k]);
fprintf('Direct TF poles vs. SS-derived poles:\n')
[pole(G_direct) pole(G_from_ss)]

%% Transfer Function to State-Space
% Conversely, `ss(G)` realizes a transfer function in (typically)
% controllable canonical form. The realization is *not unique* --
% infinitely many $(A,B,C,D)$ share the same transfer function, related
% by similarity transformations $\bar{x}=Tx$ (see
% |RepresenationTheory.m|).
G3 = tf([1 3],[1 4 6 8]);
sys_from_tf = ss(G3);
fprintf('Canonical-form realization of G3:\n')
sys_from_tf.A
sys_from_tf.B
sys_from_tf.C

%% Free (Zero-Input) Response and the State-Transition Matrix
% With $u(t)=0$, $\dot{x}=Ax$ has solution $x(t)=e^{At}x(0)$, where
% $\Phi(t)=e^{At}$ is the *state-transition matrix*. For a general input,
% the full solution is the variation-of-parameters formula:
%
% $$x(t) = e^{At}x(0) + \int_0^t e^{A(t-\tau)}Bu(\tau)\,d\tau$$
x0 = [1; 0];
t = 0:0.01:5;
[y_free,~,x_free] = initial(sys_ss,x0,t);
figure
plot(t,x_free)
legend('$x_1(t)$','$x_2(t)$','Interpreter','latex','FontSize',14)
title('Zero-Input (Free) Response from $x(0)=[1;0]$','Interpreter','latex','FontSize',20)
ylabel('$x(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
