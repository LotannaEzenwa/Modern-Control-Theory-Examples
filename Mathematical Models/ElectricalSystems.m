%% Electrical Systems
% *Kirchhoff's laws to transfer functions, and the electrical-mechanical analogy.*
%
% Ogata, _Modern Control Engineering_, Ch. 3.
%
% In this tutorial you will:
%
% * model a series RLC circuit with KVL,
% * see the force-voltage analogy with the mass-spring-damper, and
% * model an op-amp PI controller circuit.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% Electrical circuits are modeled with Kirchhoff's voltage and current
% laws. Consider a series RLC circuit driven by source voltage
% $e_i(t)$, with output taken as the voltage across the capacitor
% $e_o(t)$. KVL around the loop, with loop current $i(t)$:
%
% $L\frac{di}{dt} + Ri + \frac{1}{C}\int i\,dt = e_i$
%
% and since $e_o = \frac{1}{C}\int i\,dt$, we have $i = C\dot{e}_o$.
% Substituting:
%
% $LC\ddot{e}_o + RC\dot{e}_o + e_o = e_i$

%% Transfer Function of the RLC Circuit
% Taking the Laplace transform with zero initial conditions:
%
% $(LCs^2 + RCs + 1)E_o(s) = E_i(s) \quad\Rightarrow\quad
%   G(s) = \frac{E_o(s)}{E_i(s)} = \frac{1}{LCs^2+RCs+1}$
%
% With $L=1\,\mathrm{H}$, $R=2\,\Omega$, $C=0.5\,\mathrm{F}$:
L = 1; R = 2; C = 0.5;
G_rlc = tf(1,[L*C R*C 1])

figure
[es,ts] = step(G_rlc);
plot(ts, ones(size(ts)),'k--', ts, es,'b','LineWidth',1.3)
legend('Input voltage step $e_i$ (before)','Capacitor voltage $e_o(t)$ (after)','Interpreter','latex','FontSize',11,'Location','east')
title('Input vs. Output: Series RLC Circuit','Interpreter','latex','FontSize',16)
ylabel('amplitude','Interpreter','latex','FontSize',16)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% The Electrical-Mechanical Analogy
% The RLC circuit equation is structurally identical to the
% mass-spring-damper equation $m\ddot{x}+b\dot{x}+kx=u$ from
% |MechanicalSystems.m|, under the force-voltage analogy:
%
%   Mechanical          Electrical
%   ------------------  ------------------
%   Force, u            Voltage, e
%   Mass, m             Inductance, L
%   Damping coeff., b   Resistance, R
%   Spring const., k    Elastance, 1/C
%   Displacement, x      Charge, q
%   Velocity, dx/dt      Current, i
%
% This analogy is why the same transfer-function machinery (and the
% same MATLAB code) applies to both physical domains.

%% Operational-Amplifier Circuit
% A common building block is the inverting op-amp circuit with input
% impedance $Z_i(s)$ and feedback impedance $Z_f(s)$, giving
%
% $\frac{E_o(s)}{E_i(s)} = -\frac{Z_f(s)}{Z_i(s)}$
%
% For a PI-controller circuit, $Z_i = R_1$ (a resistor) and
% $Z_f = R_2 + \frac{1}{C_2 s}$ (a resistor and capacitor in series):
%
% $G(s) = -\frac{R_2 + \frac{1}{C_2 s}}{R_1} =
%   -\frac{R_2 C_2 s + 1}{R_1 C_2 s}$
R1 = 10e3; R2 = 20e3; C2 = 1e-6;
G_opamp = tf(-[R2*C2 1],[R1*C2 0])

figure
bode(G_opamp)
title('Op-Amp PI Circuit Frequency Response','Interpreter','latex','FontSize',20)
