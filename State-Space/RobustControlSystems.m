%% Robustness of State-Space Control Systems
% *Will the design survive a plant that differs from the model?*
%
% Ogata, _Modern Control Engineering_, Ch. 10.
%
% In this tutorial you will:
%
% * form the sensitivity $S$ and complementary sensitivity $T$ of a loop,
% * read classical gain/phase margins at the plant input, and
% * test the closed loop against plant-parameter perturbations.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% A controller designed from a nominal plant model $A,B,C$ must still
% perform acceptably when the real plant differs (parameter uncertainty,
% unmodeled dynamics). *Sensitivity* and *gain/phase margins* quantify
% this robustness for a state-feedback loop broken at the plant input.

%% Nominal Plant and LQR Design
A = [0 1; -2 -3];
B = [0; 1];
C = [1 0];
D = 0;

Q = diag([5 1]);
R = 1;
K = lqr(A,B,Q,R);
fprintf('Nominal LQR gain K = '); disp(K)
fprintf('Nominal closed-loop poles: '); disp(eig(A-B*K)')

%% Loop Gain and Sensitivity Function
% Breaking the loop at the plant input, the loop transfer function is
% $L(s)=K(sI-A)^{-1}B$. The *sensitivity function*
% $S(s)=(1+L(s))^{-1}$ measures how much output disturbances or plant
% perturbations are attenuated by feedback; the *complementary
% sensitivity* $T(s)=L(s)(1+L(s))^{-1}=1-S(s)$ measures reference/noise
% transmission. A small $\|S\|_\infty$ near crossover indicates good
% disturbance rejection; $S+T=1$ is an unavoidable design tradeoff.
sys_ol = ss(A,B,K,0);
L = tf(sys_ol);
S = feedback(1,L);
T = feedback(L,1);

w = logspace(-2,2,500);
[magS,~] = bode(S,w);
[magT,~] = bode(T,w);
figure
semilogx(w,20*log10(squeeze(magS)),'b',w,20*log10(squeeze(magT)),'r--')
legend('$|S(j\omega)|$','$|T(j\omega)|$','Interpreter','latex','FontSize',14)
title('Sensitivity and Complementary Sensitivity','Interpreter','latex','FontSize',20)
ylabel('Magnitude (dB)','Interpreter','latex','FontSize',16)
xlabel('$\omega$ (rad/s)','Interpreter','latex','FontSize',20)
grid on

%% Classical Margins at the Plant Input
[Gm,Pm,Wcg,Wcp] = margin(L);
fprintf('\nGain margin = %.2f dB, Phase margin = %.2f deg\n', ...
    20*log10(Gm), Pm)
fprintf('(LQR guarantees PM >= 60 deg, infinite upward gain margin,\n')
fprintf(' and >=50%% downward gain reduction tolerance.)\n')

%% Robustness to Parameter Perturbation
% Test closed-loop stability when the true plant's $A$ matrix deviates
% from the nominal model (e.g. an underestimated damping/stiffness
% term), while still using the gain $K$ designed from the nominal model.
perturbations = [0.7 1.0 1.3];   % multiply the (2,2) damping entry
figure
hold on
for p = perturbations
    A_true = A; A_true(2,2) = A(2,2)*p;
    sys_true_cl = ss(A_true-B*K, B, C, D);
    step(sys_true_cl)
end
hold off
legend('30% less damping','Nominal','30% more damping','Interpreter','latex','FontSize',12)
title('Closed-Loop Step Response Under Plant Perturbation','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Stability Margin Check Across Perturbations
for p = perturbations
    A_true = A; A_true(2,2) = A(2,2)*p;
    is_stable = all(real(eig(A_true-B*K))<0);
    fprintf('damping x%.1f: closed-loop poles = %s, stable = %d\n', ...
        p, mat2str(eig(A_true-B*K)',4), is_stable)
end
