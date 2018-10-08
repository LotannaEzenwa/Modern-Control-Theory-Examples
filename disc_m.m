function [ a ] = disc_m( A,B,k )
%DISC_M Summary of this function goes here
%   Detailed explanation goes here
    m = 0;
    for i = 1:k
        m = m + disc_nm(A,B,i-1,k-i);
    end
    a = m;
end

