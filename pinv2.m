function X = pinv2(A, tol)
%PINV2  Tolerance-truncated Moore-Penrose pseudoinverse via the SVD.
%   X = PINV2(A, tol) returns the pseudoinverse of A, discarding any
%   singular value whose ratio to the largest singular value does not
%   exceed tol. Truncating these near-dependent directions rejects the
%   noise-dominated subspace of the data matrices that arise in observer
%   Markov parameter (OKID) identification, giving a numerically robust
%   least-squares solve.
%
%   If tol is omitted, MATLAB's default pinv tolerance is used.
%
%   Inputs:
%     A   : matrix to pseudo-invert
%     tol : fractional singular-value threshold, 0 <= tol < 1 (e.g. 1e-5).
%           A singular value sigma_i is kept iff sigma_i/sigma_max > tol.
%
%   Output:
%     X   : pseudoinverse of A, size(A,2)-by-size(A,1)
%
%   Example:
%     Ahat = pinv2(V_bar, 1e-5);   % used as Y_bar.' * pinv2(V_bar,1e-5)

    [U, S, V] = svd(A, 'econ');
    s = diag(S);

    if isempty(s)
        X = zeros(size(A, 2), size(A, 1));
        return
    end

    if nargin < 2 || isempty(tol)
        % Absolute tolerance, matching MATLAB's pinv default.
        thresh = max(size(A)) * eps(max(s));
        keep = s > thresh;
    else
        % Relative tolerance: keep sigma_i with sigma_i/sigma_max > tol.
        keep = (s / max(s)) > tol;
    end

    sinv = zeros(size(s));
    sinv(keep) = 1 ./ s(keep);
    X = V * diag(sinv) * U';
end
