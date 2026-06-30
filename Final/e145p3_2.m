clear
M = eye(8)*100;
K = [27071.1 0 0 0 -10000 0 -3535.5 -3535.5
    0 17071.1 0 -10000 0 0 -3535.5 -3535.5
    0 0 27071.1 0 -3535.5 -3535.5 -10000 0
    0 -10000 0 17071.1 3535.5 -3535.5 -10000 0 
    -10000 0 -3535.5 3535.5 27071.1 0 0 0
    0 0 3535.5 -3535.5 0 17071.1 0 -10000
    -3535.5 -3535.5 -10000 0 0 0 27071.1 0
    -3535.5 -3535.5 0 0 0 -10000 0 17071.1];

%% Model
Ac = [zeros(8) eye(8)
    -inv(M)*K zeros(8)];

b_f = zeros(8,4);
b_v = inv(M)*b_f;
B_c = [zeros(8,4)
    b_v];

%% Visualizing the Stiffness Coupling
% The stiffness matrix K encodes how the 8 degrees of freedom are coupled;
% its off-diagonal entries show which masses exert forces on which others.
figure
imagesc(K)
axis square
colorbar
title('Stiffness Matrix $K$ (coupling structure)','Interpreter','latex','FontSize',20)
ylabel('DOF $i$','Interpreter','latex','FontSize',16)
xlabel('DOF $j$','Interpreter','latex','FontSize',16)

%% Natural Frequencies of the Undamped Model
% With $\dot{z}=A_cz$ and $A_c=\begin{bmatrix}0&I\\-M^{-1}K&0\end{bmatrix}$,
% the eigenvalues of $A_c$ are purely imaginary, $\pm j\omega_i$, where the
% natural frequencies are $\omega_i=\sqrt{\mathrm{eig}(M^{-1}K)}$.
omega_n = sort(sqrt(eig(M\K)));
figure
stem(1:length(omega_n), omega_n, 'filled', 'LineWidth', 1.5)
grid on
title('Undamped Natural Frequencies','Interpreter','latex','FontSize',20)
ylabel('$\omega_i$ (rad/s)','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('Mode number $i$','Interpreter','latex','FontSize',20)

%% Pole Map
% Confirm the modes lie on the imaginary axis (undamped: no real part).
figure
plot(real(eig(Ac)), imag(eig(Ac)), 'bx', 'MarkerSize', 10, 'LineWidth', 1.5)
grid on
axis equal
title('Poles of the Undamped Structural Model','Interpreter','latex','FontSize',18)
ylabel('$\mathrm{Im}$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$\mathrm{Re}$','Interpreter','latex','FontSize',20)