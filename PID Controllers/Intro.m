%% Introduction to PID Control
% Ogata, Modern Control Engineering, Ch. 8: PID Controllers
%
% A PID controller computes its actuating signal from the error
% $e(t)=r(t)-y(t)$ as a weighted sum of proportional, integral, and
% derivative terms:
%
% $u(t) = K_p e(t) + K_i\int_0^t e(\tau)\,d\tau + K_d\frac{de(t)}{dt}$
%
% with transfer function (often written with $K_i=K_p/T_i$,
% $K_d=K_pT_d$):
%
% $G_c(s) = K_p\left(1+\frac{1}{T_i s}+T_d s\right)$
%
% where $T_i$ is the *integral (reset) time* and $T_d$ the *derivative
% (rate) time*.

%% Plant Used Throughout This File
% $G(s) = \frac{1}{(s+1)(s+2)}$, unity feedback, step reference.
G = tf(1,conv([1 1],[1 2]));
t = 0:0.01:15;

%% Proportional (P) Control
% $G_c(s)=K_p$. Increasing $K_p$ speeds the response and reduces (but
% does not eliminate) steady-state error for a type-0 plant, at the cost
% of increased overshoot.
Kps = [2 5 15];
figure
hold on
for Kp = Kps
    T = feedback(Kp*G,1);
    [y,tt] = step(T,t);
    plot(tt,y,'DisplayName',sprintf('K_p=%d',Kp))
end
yline(1,'k--','HandleVisibility','off')
hold off
legend('Interpreter','latex','FontSize',12)
title('Proportional Control: Effect of $K_p$','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

Kp_demo = 5;
T_p = feedback(Kp_demo*G,1);
ess_p = 1 - dcgain(T_p);
fprintf('P control (Kp=%d): steady-state error = %.4f\n', Kp_demo, ess_p)

%% Proportional-Integral (PI) Control
% $G_c(s)=K_p\left(1+\frac{1}{T_is}\right)$. The integral term adds a
% pole at the origin, making the loop type-1 and driving steady-state
% step error to zero, at the cost of a slower response and reduced phase
% margin (more oscillatory) compared to P alone.
Kp_pi = 5; Ti = 2;
Gc_pi = tf(Kp_pi*[1 1/Ti],[1 0]);
T_pi = feedback(Gc_pi*G,1);
ess_pi = 1 - dcgain(T_pi);
fprintf('PI control: steady-state error = %.6f (should be ~0)\n', ess_pi)

figure
hold on
step(T_p,t)
step(T_pi,t)
hold off
legend('P only','PI','Interpreter','latex','FontSize',14)
title('P vs. PI Control','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Proportional-Derivative (PD) Control
% $G_c(s)=K_p(1+T_ds)$. The derivative term adds phase lead, improving
% damping/reducing overshoot and speeding settling, but does not affect
% steady-state error and amplifies high-frequency measurement noise.
Kp_pd = 5; Td = 0.3;
Gc_pd = tf(Kp_pd*[Td 1],1);
T_pd = feedback(Gc_pd*G,1);

figure
hold on
step(T_p,t)
step(T_pd,t)
hold off
legend('P only','PD','Interpreter','latex','FontSize',14)
title('P vs. PD Control','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Full PID Control
% Combining all three terms gives zero steady-state error (from the
% integral action) together with good transient response (from the
% derivative action).
Kp = 5; Ti = 2; Td = 0.3;
Gc_pid = tf(Kp*[Td*Ti 1 1/Ti]*Ti/Ti,[Ti 0]); % Kp*(Td*s^2 + s + 1/Ti)/s, written via Ti
Gc_pid = tf(Kp*[Td 1 1/Ti],[1 0]);
T_pid = feedback(Gc_pid*G,1);
ess_pid = 1 - dcgain(T_pid);
fprintf('PID control: steady-state error = %.6f\n', ess_pid)

figure
hold on
step(T_p,t)
step(T_pi,t)
step(T_pd,t)
step(T_pid,t)
hold off
legend('P','PI','PD','PID','Interpreter','latex','FontSize',14)
title('Qualitative Comparison of P, PI, PD, PID Control','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)
