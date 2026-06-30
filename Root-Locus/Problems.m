%% Root-Locus -- Worked Problems
% Ogata, Modern Control Engineering, Ch. 6: end-of-chapter style
% root-locus and compensator design problems.

%% Problem 1: Locate the Breakaway Point
% For $G(s) = \frac{K}{s(s+4)}$, find the breakaway point on the real
% axis and the corresponding gain.
%
% Characteristic equation: $s(s+4)+K=0 \Rightarrow K=-s^2-4s$.
% Breakaway: $\frac{dK}{ds}=-2s-4=0 \Rightarrow s=-2$.
% $K$ at breakaway: $K = -(-2)^2-4(-2) = -4+8 = 4$.
syms s K
Ksym = -(s^2+4*s);
dKds = diff(Ksym,s);
s_breakaway = double(solve(dKds==0,s));
K_breakaway = double(subs(Ksym,s,s_breakaway));
fprintf('Breakaway point s = %.4f, K = %.4f\n', s_breakaway, K_breakaway)

G1 = tf(1,[1 4 0]);
figure
rlocus(G1)
hold on
plot(real(s_breakaway),imag(s_breakaway),'ro','MarkerSize',8)
hold off
title('Problem 1: Root Locus with Breakaway Point','Interpreter','latex','FontSize',20)
grid on

%% Problem 2: Gain for a Specified Damping Ratio
% For the same $G(s) = \frac{K}{s(s+4)}$, find $K$ such that the
% closed-loop poles have $\zeta=0.707$.
%
% Closed-loop char. eq.: $s^2+4s+K=0$. Standard form
% $s^2+2\zeta\omega_n s+\omega_n^2=0 \Rightarrow 2\zeta\omega_n=4,\
% \omega_n^2=K$. With $\zeta=0.707$: $\omega_n = 4/(2\times0.707) =
% 2.828$, so $K=\omega_n^2 \approx 8$.
zeta2 = 0.707;
wn2 = 4/(2*zeta2);
K2 = wn2^2;
fprintf('Required K for zeta=0.707: K = %.4f\n', K2)

T2 = feedback(K2*G1,1);
poles_2 = pole(T2)
info2 = stepinfo(T2)

figure
step(T2)
title('Problem 2: Step Response at $\zeta=0.707$ Design Point','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Problem 3: Lead Compensator for a Settling-Time Spec
% For $G(s) = \frac{K}{s(s+4)}$, design a lead compensator
% $G_c(s)=\frac{s+z_c}{s+p_c}$ so the dominant closed-loop poles achieve
% $\zeta=0.5$ and $\omega_n=6$. (Note $\omega_n=4$ would land exactly on
% the uncompensated locus, requiring no lead; $\omega_n=6$ sits to the
% left of it, so a genuine lead network is needed.)
zeta3 = 0.5; wn3 = 6;
sd3 = -zeta3*wn3 + 1j*wn3*sqrt(1-zeta3^2);

ol_poles3 = [0 -4];
phase_G3 = -sum(angle(sd3 - ol_poles3))*180/pi;   % angle of G(sd3)
phi_lead3 = mod((-180 - phase_G3) + 360, 360);    % lead phase to add
fprintf('Desired pole: %.4f + %.4fj\n', real(sd3), imag(sd3))
fprintf('Open-loop phase at s_d = %.2f deg, lead to supply = %.2f deg\n', ...
    phase_G3, phi_lead3)

zc3 = -real(sd3);   % place zero under the desired pole's real part
angle_zero3 = angle(sd3 + zc3)*180/pi;
angle_pole3 = angle_zero3 - phi_lead3;
pc3 = -real(sd3) + imag(sd3)/tand(angle_pole3);

Gc3 = tf([1 zc3],[1 pc3]);
G3 = tf(1,[1 4 0]);
Kc3 = 1/abs(evalfr(Gc3*G3,sd3));
fprintf('Lead compensator: zero at %.4f, pole at %.4f, gain Kc = %.4f\n', -zc3, -pc3, Kc3)

G3_comp = series(Kc3*Gc3,G3);
T3_comp = feedback(G3_comp,1);

% Verify a closed-loop pole lands near the target s_d.
cl3 = pole(T3_comp);
[~,i3] = min(abs(cl3 - sd3));
fprintf('Nearest closed-loop pole to s_d: %.4f + %.4fj\n', real(cl3(i3)), imag(cl3(i3)))

figure
step(T3_comp)
title('Problem 3: Lead-Compensated Step Response','Interpreter','latex','FontSize',20)
ylabel('$y(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)
