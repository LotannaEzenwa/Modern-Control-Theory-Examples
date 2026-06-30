function [Y_bar, V_bar] = YV_Form_nonzero(u, y, p)
%YV_FORM_NONZERO  Build the OKID regression data matrices.
%   [Y_bar, V_bar] = YV_FORM_NONZERO(u, y, p) forms the output stack
%   Y_bar and the data matrix V_bar used to identify the first p observer
%   Markov parameters of a linear discrete-time system from a single
%   input/output record, allowing for a nonzero (but unknown) initial
%   condition.
%
%   Inputs:
%     u : m-by-N input history  (m inputs,  N samples)
%     y : q-by-N output history (q outputs, N samples)
%     p : number of observer Markov parameters to regress for
%
%   Outputs:
%     Y_bar : (N-p)-by-q   output stack, so that Y_bar.' is q-by-(N-p)
%     V_bar : (m + p*(m+q))-by-(N-p) regression matrix
%
%   The observer (Kalman-filter) form of a discrete LTI system is
%       x_{k+1} = (A+GC) x_k + [B+GD, -G] v_k,   v_k = [u_k; y_k]
%       y_k     = C x_k + D u_k,
%   so with a deadbeat observer (eig(A+GC)=0) and k >= p the free response
%   C*(A+GC)^k*x0 has died out and
%       y_k = Ybar_0 u_k + sum_{i=1}^{p} Ybar_i v_{k-i}.
%   Stacking this over k = p..N-1 gives  Y_bar.' = Ybar * V_bar, which the
%   caller solves for the observer Markov parameter row block Ybar via
%       Ybar = Y_bar.' * pinv2(V_bar, tol).
%   Because the regression starts at k = p, the unknown initial condition
%   contributes nothing -- hence the "_nonzero" suffix.
%
%   See also PINV2, RECOVER_SYSMP.

    [m, N]  = size(u);
    [q, Ny] = size(y);
    if Ny ~= N
        error('YV_Form_nonzero:size', ...
              'u and y must have the same number of samples (columns).');
    end

    L = N - p;                       % number of usable regression columns
    Y_bar = zeros(L, q);
    V_bar = zeros(m + p*(m+q), L);

    for j = 1:L
        k = p + j;                   % current time index, k = p+1 .. N
        Y_bar(j, :) = y(:, k).';
        col = u(:, k);               % Ybar_0 multiplies the current input
        for i = 1:p
            col = [col; u(:, k-i); y(:, k-i)];   % v_{k-i} = [u_{k-i}; y_{k-i}]
        end
        V_bar(:, j) = col;
    end
end
