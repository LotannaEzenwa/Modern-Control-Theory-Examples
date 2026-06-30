function RSMP = recover_SYSMP(Ybar, n_mp, q, m)
%RECOVER_SYSMP  Recover system Markov parameters from observer ones.
%   RSMP = RECOVER_SYSMP(Ybar, n_mp, q, m) converts the observer Markov
%   parameter row block Ybar (as produced by the OKID least-squares step
%   Ybar = Y_bar.' * pinv2(V_bar, tol)) into the first n_mp system Markov
%   parameters Y_0, Y_1, ..., Y_{n_mp-1} of the underlying plant.
%
%   Inputs:
%     Ybar : q-by-(m + p*(m+q)) observer Markov parameter block, laid out
%            as [Ybar_0, Ybar_1, ..., Ybar_p] where Ybar_0 = D (q-by-m)
%            and each Ybar_i = [Ybar_i^(1), Ybar_i^(2)] with Ybar_i^(1)
%            (q-by-m) the input part and Ybar_i^(2) (q-by-q) the output
%            part.
%     n_mp : number of system Markov parameters to return
%     q    : number of outputs
%     m    : number of inputs
%
%   Output:
%     RSMP : q-by-(m*n_mp) block row [Y_0, Y_1, ..., Y_{n_mp-1}];
%            for a single-input system this is simply q-by-n_mp.
%
%   With the observer defined by Abar = A+GC and Bbar = [B+GD, -G], so that
%   Ybar_i^(2) = -C*Abar^(i-1)*G, the system Markov parameters satisfy
%   (Juang, Applied System Identification):
%       Y_0 = Ybar_0 = D
%       Y_k = Ybar_k^(1) + sum_{i=1}^{min(k,p)} Ybar_i^(2) Y_{k-i},
%   where Ybar_k^(1) is taken as zero for k > p (the observer is finite).
%
%   See also YV_FORM_NONZERO, PINV2.

    p = (size(Ybar, 2) - m) / (m + q);
    if p ~= floor(p) || p < 0
        error('recover_SYSMP:size', ...
              'Ybar width is inconsistent with the given q and m.');
    end

    % Split Ybar into Ybar_0 = D and the i = 1..p blocks.
    Y0  = Ybar(:, 1:m);
    Yb1 = cell(1, p);   % input  parts Ybar_i^(1)  (q-by-m)
    Yb2 = cell(1, p);   % output parts Ybar_i^(2)  (q-by-q)
    for i = 1:p
        base   = m + (i-1)*(m+q);
        Yb1{i} = Ybar(:, base + (1:m));
        Yb2{i} = Ybar(:, base + m + (1:q));
    end

    % Recursion. Y{k+1} holds the system Markov parameter Y_k.
    Y = cell(1, n_mp);
    Y{1} = Y0;
    for k = 1:n_mp-1
        if k <= p
            Yk = Yb1{k};
        else
            Yk = zeros(q, m);
        end
        for i = 1:min(k, p)
            Yk = Yk + Yb2{i} * Y{k-i+1};
        end
        Y{k+1} = Yk;
    end

    % Assemble as a block row [Y_0, Y_1, ..., Y_{n_mp-1}].
    RSMP = zeros(q, m*n_mp);
    for k = 1:n_mp
        RSMP(:, (k-1)*m + (1:m)) = Y{k};
    end
end
