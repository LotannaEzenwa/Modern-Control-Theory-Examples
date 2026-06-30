%% Automatic Control Systems: Open-Loop vs. Closed-Loop
% Ogata, Modern Control Engineering, Ch. 1-2
%
% An *open-loop* control system uses a controller and process (plant)
% in a forward path only -- the output is not measured or compared to
% the reference, so accuracy depends entirely on calibration and is not
% corrected for disturbances:
%
% $Y(s) = G_c(s)G_p(s)R(s)$
%
% A *closed-loop* (feedback) system measures the output, compares it
% to the reference through a summing junction, and drives the
% controller from the resulting error $E(s)=R(s)-H(s)Y(s)$:
%
% $Y(s) = \frac{G_c(s)G_p(s)}{1+G_c(s)G_p(s)H(s)}R(s)$
%
% Feedback trades some open-loop simplicity for reduced sensitivity to
% plant variations and disturbances, at the cost of potential
% instability if not designed carefully.

%% Plant and Controller
% Consider a plant $G_p(s) = \frac{1}{s+1}$ controlled by a simple
% proportional controller $G_c(s) = K$, with unity feedback
% $H(s)=1$.
Gp = tf(1,[1 1]);
K = 4;
Gc = K;

T_open = series(Gc,Gp);          % open-loop output, no feedback
T_closed = feedback(series(Gc,Gp),1);

figure
hold on
step(T_open)
step(T_closed)
hold off
legend('Open-loop','Closed-loop (unity feedback)','Interpreter','latex','FontSize',14)
title('Open-Loop vs. Closed-Loop Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% Note the open-loop system does not track $r=1$: its unit-step output
% settles to $KG_p(0)=4$, with nothing to correct it toward the
% reference. The closed-loop system's steady-state error is instead
% reduced by the loop gain:
%
% $e_{ss} = \frac{1}{1+KG_p(0)H(0)} = \frac{1}{1+4} = 0.2$
ess_closed = 1/(1+K*dcgain(Gp))

%% Disturbance Rejection
% A key advantage of feedback is disturbance rejection. Add a
% disturbance $D(s)$ entering at the plant input:
%
% $Y(s) = \frac{G_c G_p}{1+G_cG_pH}R(s) +
%   \frac{G_p}{1+G_cG_pH}D(s)$
%
% As the loop gain $G_cG_pH$ grows, the disturbance's contribution to
% $Y(s)$ shrinks, even though $G_p$ itself is unchanged.
T_dist_open = Gp;                          % open-loop: D feeds Y directly through Gp
T_dist_closed = feedback(Gp,Gc);           % closed-loop: D path attenuated by loop gain

figure
hold on
step(T_dist_open)
step(T_dist_closed)
hold off
legend('Open-loop disturbance response','Closed-loop disturbance response','Interpreter','latex','FontSize',12)
title('Disturbance Rejection via Feedback','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)
