clear
M = [ 1 1 
    -1 -1];
u = [0];
x = [1];

for i = 1:10
    x(i+1) = x(i) + u(i);
    u(i+1) = -x(i) + -u(i);
    
end

z = [x; u]

%% Visualizing the State Trajectory
% The update is $z_{k+1}=Mz_k$ with $M=\begin{bmatrix}1&1\\-1&-1\end{bmatrix}$.
% Since $M^2=0$ (M is nilpotent), the state is driven to the origin in a
% single step -- a deadbeat response, visible as both sequences collapsing
% to zero after $k=1$.
figure
stairs(0:length(x)-1, x, 'b-o', 'LineWidth', 1.5)
hold on
stairs(0:length(u)-1, u, 'r-s', 'LineWidth', 1.5)
hold off
grid on
legend('$x_k$', '$u_k$', 'Interpreter', 'latex', 'FontSize', 14)
title('Deadbeat State Trajectory $z_{k+1}=Mz_k$', 'Interpreter', 'latex', 'FontSize', 20)
ylabel('$z_k$', 'Interpreter', 'latex', 'FontSize', 20)
set(get(gca, 'YLabel'), 'Rotation', 0, 'HorizontalAlignment', 'right')
xlabel('$k$', 'Interpreter', 'latex', 'FontSize', 20)