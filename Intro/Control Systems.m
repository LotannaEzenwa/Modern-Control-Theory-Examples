%% Introduction to Control Systems
% Ogata, Modern Control Engineering, Ch. 1: Introduction to Control
% Systems
%
% A *control system* is an interconnection of components that regulates
% its own behavior or that of another system, typically to achieve a
% desired output. The two fundamental architectures are *open-loop* and
% *closed-loop (feedback)* control.

%% Open-Loop Control
% An open-loop system computes its actuating signal from the reference
% input alone, with no measurement of the actual output:
%
% $$U(s) = G_cR(s)$$
%
% $$Y(s) = G(s)G_cR(s)$$
%
% Open-loop control is simple and stable (no feedback loop to
% destabilize), but it has *no mechanism to correct for disturbances or
% plant uncertainty* -- the output tracks the reference only as well as
% the model $G(s)$ is known.

%% Closed-Loop (Feedback) Control
% A closed-loop system measures the output $y(t)$, forms the error
% $e(t)=r(t)-y(t)$, and drives the actuator from that error:
%
% $$\frac{Y(s)}{R(s)} = \frac{G_c(s)G(s)}{1+G_c(s)G(s)H(s)}$$
%
% where $H(s)$ is the sensor/feedback transfer function (often $H=1$ for
% unity feedback). Feedback trades some complexity and the possibility of
% instability for the ability to *automatically correct for disturbances
% and modeling error*, provided the loop is properly designed.

%% Example: Plant Subject to a Load Disturbance
% Compare open-loop and closed-loop (unity-feedback, proportional
% control) responses to a step reference *and* a step disturbance
% entering at the plant input, for $G(s)=\frac{1}{s+1}$.
G = tf(1,[1 1]);
Kp = 9;   % proportional gain for the closed-loop case

% Open-loop: u = Kp*r directly, no correction for disturbance d
sys_ol_r = Kp*G;            % reference path
sys_ol_d = G;                % disturbance path (unity gain into plant)

% Closed-loop: e = r - y, u = Kp*e + d enters at plant input
sys_cl_r = feedback(Kp*G,1);          % reference -> output
sys_cl_d = feedback(G,Kp);            % disturbance -> output

t = 0:0.01:5;
r = ones(size(t));
d_step_time = 2;   % disturbance step applied at t=2
d = double(t>=d_step_time);

y_ol = lsim(sys_ol_r,r,t) + lsim(sys_ol_d,d,t);
y_cl = lsim(sys_cl_r,r,t) + lsim(sys_cl_d,d,t);

figure
plot(t,y_ol,'b',t,y_cl,'r')
hold on
xline(d_step_time,'k:','disturbance applied')
hold off
legend('Open-loop','Closed-loop (feedback)','Interpreter','latex','FontSize',14)
title('Disturbance Rejection: Open-Loop vs. Closed-Loop','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% The open-loop response permanently shifts after the disturbance hits
% (nothing measures or corrects for it); the closed-loop response is
% pulled back toward the reference because the error signal $e=r-y$
% picks up the deviation and the controller acts on it.
ess_ol = y_ol(end) - 1;
ess_cl = y_cl(end) - 1;
fprintf('Steady-state error after disturbance -- open-loop: %.4f, closed-loop: %.4f\n', ...
    ess_ol, ess_cl)

%% Canonical Feedback Block Diagram
% The standard unity-feedback diagram analyzed throughout this repository
% is: reference $R(s)\to$ summing junction (error $E=R-Y$) $\to$
% controller $G_c(s)\to$ plant $G(s)\to$ output $Y(s)$, fed back to the
% summing junction. MATLAB's `feedback` function implements exactly this
% reduction:
%
% $$T(s) = \mathrm{feedback}(G_cG,1) = \frac{G_cG}{1+G_cG}$$
Gc = tf(Kp,1);
T = feedback(Gc*G,1);
figure
step(T)
title('Canonical Closed-Loop Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% This canonical loop -- and its state-space, root-locus, and
% frequency-domain analysis counterparts -- is developed in depth in the
% topic directories of this repository: |Mathematical Models/|,
% |Transient and Steady-State/|, |Root-Locus/|, |Frequency-Response/|,
% |PID Controllers/|, and |State-Space/|.
