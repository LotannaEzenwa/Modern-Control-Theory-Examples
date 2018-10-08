function [ Q ] = pmatrix( C, A, B, n )
%PMATRIX Creates the P matrix detailed of the ILC
%   C = Observability, A = Dynamics, B = Input, n = timesteps
    Q = zeros(n,n);
    for i = 1:n
        row = zeros(1,n);
        for j = 0:i-1
            row(j+1) = C*(A^(i-j-1))*B;
        end
        Q(i,:) = row;
    end

end

