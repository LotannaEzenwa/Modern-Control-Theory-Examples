%% Servo & Tracking -- Worked Problems
% *Practice: integral servo design and steady-state tracking limits.*
%
% Ogata, _Modern Control Engineering_, Ch. 10 (servo systems).
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.

%% Problem 1: Integral servo to a pole-placement spec
% For $G(s)=\frac{1}{(s+1)(s+3)}$, design integral-plus-state feedback that
% places the augmented poles at $-4,-5,-6$, and confirm zero steady-state
% error to a step *and* rejection of a constant input disturbance.
A = [0 1; -3 -4];  B = [0;1];  C = [1 0];
Aa = [A, zeros(2,1); -C, 0];  Ba = [B;0];
Ka = place(Aa, Ba, [-4 -5 -6]);
K = Ka(1:2);  Ki = Ka(3);
Acl = [A - B*K, -B*Ki; -C, 0];  Ccl = [C, 0];

sys_r = ss(Acl, [0;0;1], Ccl, 0);      % reference -> output
sys_d = ss(Acl, [B;0],  Ccl, 0);       % input disturbance -> output
fprintf('Problem 1: step tracking error = %.2e, disturbance DC gain = %.2e\n', ...
    1 - dcgain(sys_r), dcgain(sys_d))

t = 0:0.01:6;
figure
step(sys_r, t)
hold on
step(sys_d, t)
yline(1,'k:','HandleVisibility','off')
hold off
grid on
legend('Reference tracking','Input-disturbance response','Interpreter','latex','FontSize',12)
title('Problem 1: Integral Servo (tracks and rejects)','Interpreter','latex','FontSize',15)
ylabel('$y$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$t$ (s)','Interpreter','latex','FontSize',16)

%% Problem 2: what a type-1 servo can and cannot track
% The integral servo above is *type 1*: zero error to a step, but a constant
% error to a *ramp*. Confirm it by driving the same servo with a ramp
% reference and measuring the steady lag.
tr = 0:0.01:10;
y_ramp = lsim(sys_r, tr, tr);          % ramp reference r(t) = t
figure
plot(tr, tr,'k--', tr, y_ramp,'b','LineWidth',1.2)
grid on
legend('Ramp reference $r=t$','Servo output','Interpreter','latex','FontSize',12,'Location','northwest')
title('Problem 2: a Type-1 Servo Lags a Ramp by a Constant','Interpreter','latex','FontSize',15)
ylabel('$y$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$t$ (s)','Interpreter','latex','FontSize',16)
fprintf('Problem 2: steady ramp-tracking lag = %.4f\n', tr(end) - y_ramp(end))

%% Try it yourself
% * Add a second integrator (a type-2 servo) and notice the ramp error
%   vanish too -- at the cost of a harder-to-stabilize loop.
% * Move the augmented poles faster and watch both tracking and disturbance
%   rejection speed up together.
