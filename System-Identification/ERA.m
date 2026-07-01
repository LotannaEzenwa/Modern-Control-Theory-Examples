%% System Identification III: The Eigensystem Realization Algorithm (ERA)
% *From an impulse response to a state-space model $(A,B,C)$.*
%
% |Intro.m| and |OKID.m| produce Markov parameters -- the impulse response.
% ERA (Juang & Pappa) turns that sequence into an actual state-space
% realization, and along the way *reveals the model order* from the rank of
% a Hankel matrix. You will:
%
% * build the block Hankel matrices $H(0)$ and $H(1)$ from Markov parameters,
% * read the order off the singular values, and
% * recover $(A,B,C)$ and confirm its poles and response match the truth.
%
% Run with |publish('ERA.m')|, or step through with *Ctrl+Enter*.

%% Markov parameters to work from
% Take a known 3rd-order discrete system and list its impulse response
% $Y_k=CA^{k-1}B$. (In practice these come from OKID.)
A_true = [0.5 0.6 0; -0.3 0.4 0.2; 0 -0.1 0.7];
B_true = [1; 0; 0.5];
C_true = [1 0 -1];
nt = size(A_true,1);

L = 40;
Y = zeros(1,L);
for k = 1:L
    Y(k) = C_true*A_true^(k-1)*B_true;
end

%% Build the block Hankel matrices
% With $H(0)_{ij}=Y_{i+j-1}$ and $H(1)_{ij}=Y_{i+j}$ (a one-step shift):
s  = 15;                          % Hankel size (>= system order)
H0 = hankel(Y(1:s),   Y(s:2*s-1));
H1 = hankel(Y(2:s+1), Y(s+1:2*s));

%% The order is the rank of H(0)
% *Notice that* only a few singular values are meaningfully nonzero -- their
% count is the model order $n$. Everything below the "elbow" is numerical
% dust.
sv = svd(H0);
figure
semilogy(sv,'o-','LineWidth',1.2)
grid on
title('Hankel Singular Values Reveal the Order','Interpreter','latex','FontSize',15)
ylabel('$\sigma_i$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$i$','Interpreter','latex','FontSize',16)

n = sum(sv > 1e-8*sv(1));         % numerical rank
fprintf('Detected model order n = %d (true = %d)\n', n, nt)

%% Recover (A, B, C)
% Truncate the SVD $H(0)=R\Sigma V^T$ to order $n$ and apply the ERA
% formulas:
%
% $$ \hat{A}=\Sigma_n^{-1/2}R_n^TH(1)V_n\Sigma_n^{-1/2},\quad
%    \hat{B}=\Sigma_n^{1/2}V_n^Te_1,\quad
%    \hat{C}=e_1^TR_n\Sigma_n^{1/2}. $$
[R,S,V] = svd(H0);
Rn = R(:,1:n);  Sn = S(1:n,1:n);  Vn = V(:,1:n);
Sh = Sn^(1/2);  Si = Sn^(-1/2);
Ah = Si*Rn.'*H1*Vn*Si;
Bh = Sh*Vn(1,:).';
Ch = Rn(1,:)*Sh;

%% Check: same poles, same response
% The realization is *not* unique (any similarity transform gives the same
% input/output behavior), so we compare the invariants -- the *poles* and
% the *impulse response* -- not the matrices themselves.
fprintf('True poles:      %s\n', mat2str(sort(eig(A_true)).',3))
fprintf('Recovered poles: %s\n', mat2str(sort(eig(Ah)).',3))

Yhat = zeros(1,L);
for k = 1:L
    Yhat(k) = Ch*Ah^(k-1)*Bh;
end
figure
stem(0:L-1, Y,'b','filled')
hold on
stem(0:L-1, Yhat,'r')
hold off
grid on
legend('True impulse response','ERA realization','Interpreter','latex','FontSize',13)
title('ERA Reproduces the Impulse Response','Interpreter','latex','FontSize',15)
ylabel('$Y_k$','Interpreter','latex','FontSize',16); set(get(gca,'YLabel'),'Rotation',0)
xlabel('$k$','Interpreter','latex','FontSize',16)
fprintf('Impulse-response reconstruction error: %.2e\n', norm(Yhat-Y)/norm(Y))

%% Try it yourself
% * Add noise (|Y = Y + 1e-3*randn(1,L);|) and watch the singular values
%   stop dropping sharply -- the elbow blurs and choosing |n| becomes a
%   judgement call.
% * Force |n = 2| and see ERA return the best 2nd-order approximation of
%   this 3rd-order system.
%
% *The full pipeline:* OKID (|OKID.m|) delivers Markov parameters even for
% ringing systems; ERA turns them into $(A,B,C)$. Together they take you
% from raw data to a usable state-space model.

%% Summary
% * ERA builds Hankel matrices from Markov parameters; their rank is the
%   model order.
% * A truncated SVD yields a state-space realization $(A,B,C)$.
% * Poles and impulse response are recovered (up to noise); the internal
%   state coordinates are arbitrary.
