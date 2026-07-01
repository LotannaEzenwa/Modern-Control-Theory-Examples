%% Digital Control II: A PID Controller in Discrete Time
% *From the textbook PID transfer function to a difference equation a
% microcontroller can run.*
%
% Ogata, _Modern Control Engineering_, digital PID.
%
% A real PID loop runs on a computer: every sample it reads the error,
% updates its integral and derivative estimates, and writes a new command.
% In this tutorial you will:
%
% * discretize a continuous PID controller with |c2d|,
% * read off the resulting *difference equation*,
% * close the loop in discrete time and compare with the continuous design, and
% * see how the sample time changes the achievable performance.
%
% Run with |publish('DiscretePID.m')|, or step through with *Ctrl+Enter*.

%% Plant and a continuous PID design
% Plant $G(s)=\dfrac{1}{(s+1)(s+3)}$. We first design a continuous PID
% $G_c(s)=K_p+K_i/s+K_ds$ that gives a good continuous response.
G  = tf(1,conv([1 1],[1 3]));
Kp = 30; Ki = 40; Kd = 5;
Cc = pid(Kp,Ki,Kd);
Tc = feedback(Cc*G,1);

%% Discretize the controller (emulation design)
% The *emulation* workflow designs in continuous time, then discretizes
% the controller. Tustin is the usual choice for PID: it preserves the
% integrator and approximates the phase well.
T  = 0.05;                   % sample time (seconds)
Cd = c2d(Cc, T, 'tustin');
fprintf('Discrete PID controller C(z):\n'); Cd   %#ok<NOPTS>

%% The difference equation the computer actually runs
% The numerator/denominator of $C(z)$ are the coefficients of the
% recursion executed every sample:
%
% $$ u_k = b_1 e_k + b_2 e_{k-1} + b_3 e_{k-2} - a_2 u_{k-1} - a_3 u_{k-2}. $$
[bz,az] = tfdata(Cd,'v');
fprintf('u[k] = %.3f*e[k] %+.3f*e[k-1] %+.3f*e[k-2] %+.3f*u[k-1] %+.3f*u[k-2]\n', ...
    bz(1),bz(2),bz(3), -az(2), -az(3))

%% Close the loop in discrete time
% Discretize the plant too -- with ZOH, since a physical hold sits between
% the computer and the plant -- and close the loop.
Gd = c2d(G, T, 'zoh');
Td = feedback(Cd*Gd, 1);

figure
step(Tc,'b', Td,'r--')
legend('Continuous PID','Digital PID (Tustin, $T=0.05$)','Interpreter','latex','FontSize',12)
title('Continuous vs. Digital PID','Interpreter','latex','FontSize',15)
ylabel('$y$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$t$','Interpreter','latex','FontSize',16)

%% Before vs. after: the sample time matters
% Emulation only works if you sample fast enough. Re-discretize at a much
% coarser rate and watch the extra phase lag eat the stability margin.
T2  = 0.4;
Cd2 = c2d(Cc,T2,'tustin');  Gd2 = c2d(G,T2,'zoh');
Td2 = feedback(Cd2*Gd2,1);
figure
step(Td,'r--', Td2,'k')
legend('Fast sampling ($T=0.05$)','Coarse sampling ($T=0.4$)','Interpreter','latex','FontSize',12)
title('Digital PID: Fast vs. Coarse Sampling','Interpreter','latex','FontSize',15)
ylabel('$y$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$t$','Interpreter','latex','FontSize',16)

%% Summary
% * Design continuous, then |c2d(C,T,'tustin')| to emulate the PID digitally.
% * $C(z)$ is just a difference equation in past errors and commands.
% * Sample fast: coarse sampling adds phase lag and can destabilize an
%   otherwise healthy design.
%
% *Next:* |DeadbeatControl.m| uses the discrete domain to settle in a
% finite number of steps -- something continuous control cannot do.

%% Try it yourself
% * Rebuild the controller with |'zoh'| instead of |'tustin'| and notice the
%   integrator approximation shift slightly at the fast rate.
% * Push |T2| past the point where the coarse-sampled loop goes unstable and
%   watch the step response diverge.
