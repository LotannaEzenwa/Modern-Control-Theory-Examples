% This function converts a non-positie definite symmteric matrix into a
% positive-definite matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Usage : Aspd = Spd_Mat(A)
% Where : A is 2D (2x2 or 3x3 ...) symmetric matrix
%         Aspd returns positive-definite matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Written By: Muhammad Asim Mubeen
%             University at Albany
%             Albany New York
%             &
%             Nathan Kline Institute for Psychiatric Research
%             Orangeburg New York.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Aspd = Spd_Mat(A)


if nargin==0 
    disp('No argument provided');
    disp('Please provide a symmetric matrix... (2x2 or 3x3 ...) as an argument');
    return;
end

% Making sure matrix A is symmetric 
size_A = size(A);

isSym=@(x) isequal(x,x.'); % Check weather the matrix is symmetric
sn=isSym(A);

if size_A(1) ~= size_A(2)  || max(size_A) < 2 || sn == 0
    
    disp('Please provide a symmetric matrix... (2x2 or 3x3 ...)');
    return;
    
end

[R,p]=chol(A);


if p > 0
    
    [Vec,Val] = eig(A);
    Val(Val==0) = eps; % making zero eigenvalues non-zero
    
    nidx = find(Val<0);
    Val(nidx) = -Val(nidx); % making negative eigenvalues positive
    
    Aspd = Vec * Val * Vec';
    
else
    Aspd = A;
    disp('Provided matrix is aleready positive Definite');
end
return
