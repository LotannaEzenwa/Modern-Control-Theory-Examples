%% State-Space Representation Theory: Canonical Forms
% Ogata, Modern Control Engineering, Ch. 9: Controllable, Observable, and
% Diagonal Canonical Forms
%
% For a SISO transfer function
%
% $$\frac{Y(s)}{U(s)} = \frac{b_0s^n+b_1s^{n-1}+\cdots+b_n}
%   {s^n+a_1s^{n-1}+\cdots+a_n}$$
%
% there are several standard state-space realizations, related to each
% other by a similarity transformation $\bar{x}=Tx$ (equivalently
% $A\to T^{-1}AT,\ B\to T^{-1}B,\ C\to CT$), which leaves the transfer
% function -- and hence the input-output behavior -- unchanged.

G = tf([1 6 12],[1 6 11 6]);
[num,den] = tfdata(G,'v');
fprintf('G(s) numerator: '); disp(num)
fprintf('G(s) denominator: '); disp(den)

%% Controllable Canonical Form
% Useful for pole-placement design (the structure makes Ackermann's
% formula transparent). For $a_1,a_2,a_3$ the denominator coefficients
% (monic, $a_0=1$):
%
% $$A_{cc}=\begin{bmatrix}-a_1&-a_2&-a_3\\1&0&0\\0&1&0\end{bmatrix},\quad
%   B_{cc}=\begin{bmatrix}1\\0\\0\end{bmatrix}$$
%
% $$C_{cc}=\begin{bmatrix}b_1&b_2&b_3\end{bmatrix},\quad b_i=\text{(numerator
% coefficients of the same order, after removing any }b_0\text{ direct
% term)}$$
a1 = den(2); a2 = den(3); a3 = den(4);
b1 = num(2)-num(1)*a1;
b2 = num(3)-num(1)*a2;
b3 = num(4)-num(1)*a3;
A_cc = [-a1 -a2 -a3; 1 0 0; 0 1 0];
B_cc = [1;0;0];
C_cc = [b1 b2 b3];
D_cc = num(1);
sys_cc = ss(A_cc,B_cc,C_cc,D_cc);

%% Observable Canonical Form
% The transpose structure of controllable canonical form -- useful for
% observer (Luenberger) design by duality:
%
% $$A_{oc}=\begin{bmatrix}-a_1&1&0\\-a_2&0&1\\-a_3&0&0\end{bmatrix},\quad
%   B_{oc}=\begin{bmatrix}b_1\\b_2\\b_3\end{bmatrix},\quad
%   C_{oc}=\begin{bmatrix}1&0&0\end{bmatrix}$$
A_oc = [-a1 1 0; -a2 0 1; -a3 0 0];
B_oc = [b1;b2;b3];
C_oc = [1 0 0];
D_oc = num(1);
sys_oc = ss(A_oc,B_oc,C_oc,D_oc);

%% Diagonal (Modal) Canonical Form
% When the poles $p_1,\dots,p_n$ are distinct, partial-fraction expansion
%
% $$\frac{Y(s)}{U(s)}=b_0+\frac{c_1}{s-p_1}+\frac{c_2}{s-p_2}+\frac{c_3}{s-p_3}$$
%
% gives a diagonal $A$ whose entries are the poles, decoupling the
% states -- each $\dot{x}_i=p_ix_i+u$ evolves independently.
[r_pf,p_pf,k_pf] = residue(num,den);
A_diag = diag(p_pf);
B_diag = ones(length(p_pf),1);
C_diag = r_pf';
D_diag = sum(k_pf);
sys_diag = ss(A_diag,B_diag,C_diag,D_diag);

%% Verifying Equivalence
% All three realizations -- and the original `tf`-derived `ss(G)` --
% share the same transfer function and poles, confirming they are
% similar representations of the same input-output system.
fprintf('Poles -- controllable: '); disp(pole(sys_cc)')
fprintf('Poles -- observable:   '); disp(pole(sys_oc)')
fprintf('Poles -- diagonal:     '); disp(pole(sys_diag)')

figure
hold on
step(sys_cc)
step(sys_oc,'--')
step(sys_diag,':')
hold off
legend('Controllable canonical','Observable canonical','Diagonal canonical', ...
    'Interpreter','latex','FontSize',14)
title('Equivalent Realizations: Identical Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% The Similarity Transformation Matrix
% Between controllable and observable canonical form realizations of the
% *same* system, $T$ can be found by matching the controllability
% matrices: $T = \mathcal{C}_{cc}\mathcal{C}_{oc}^{-1}$ is generally not
% applicable directly between distinct canonical types without going
% through a common basis; here we instead verify the diagonal form's
% transformation directly via the eigenvector matrix of $A_{cc}$.
[V_eig,~] = eig(A_cc);
T = V_eig;
A_check = T\A_cc*T;
fprintf('T^{-1}*A_cc*T (should match diag(poles), up to reordering):\n')
disp(real(A_check))
