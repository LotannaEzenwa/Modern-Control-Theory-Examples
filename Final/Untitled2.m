%% E145 Final -- Scratch Problem
% NOTE: this file had no descriptive name and no recoverable original
% problem statement (it was found duplicated from an unrelated script).
% The following is an inferred, plausible continuation problem in the
% style of the other Final/e145p*.m files -- least-squares system
% identification of a discrete-time impulse response from noisy
% input/output data -- not the original assignment text.

%% True (Hidden) System
% Generate noisy input/output data from a known discrete-time plant; the
% identification procedure below pretends only $u_k,y_k$ are known.
A_c = [0 1; -1 0];
B_c = [0;1];
C = [1 0];
D = 0;
T = 0.2;
sys_d = c2d(ss(A_c,B_c,C,D),T);
A = sys_d.A; B = sys_d.B; Cm = sys_d.C; Dm = sys_d.D;

N = 200;
u = randn(1,N);
[y,~] = dlsim(A,B,Cm,Dm,u);
y = y' + 0.01*randn(1,N);   % measurement noise

%% Least-Squares Markov Parameter Identification
% For a causal, zero-initial-condition (or long enough to neglect IC
% transients) SISO system, the output is a convolution sum
%
% $$y_k = \sum_{i=0}^{p} h_i u_{k-i}$$
%
% Stacking this over $k=p+1,\dots,N$ gives a linear regression
% $y=\Phi h$ solvable by least squares for the first $p+1$ Markov
% (impulse-response) parameters $h_0,\dots,h_p$.
p = 10;
Phi = zeros(N-p,p+1);
y_reg = y(p+1:N)';
for k = 1:N-p
    Phi(k,:) = u(p+k:-1:k);
end
h_hat = Phi\y_reg;

%% True Markov Parameters for Comparison
h_true = zeros(p+1,1);
h_true(1) = Dm;
for i = 1:p
    h_true(i+1) = Cm*A^(i-1)*B;
end

figure
stem(0:p,h_true,'b','filled')
hold on
stem(0:p,h_hat,'r')
hold off
legend('True $h_i$','Identified $\hat{h}_i$','Interpreter','latex','FontSize',14)
title('Scratch Problem: Least-Squares Markov Parameter ID','Interpreter','latex','FontSize',18)
ylabel('$h_i$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$i$','Interpreter','latex','FontSize',20)

err = norm(h_hat-h_true)/norm(h_true);
fprintf('Relative identification error = %.4f\n', err)
