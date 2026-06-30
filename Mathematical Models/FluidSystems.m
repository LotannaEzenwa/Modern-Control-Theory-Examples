%% Fluid (Liquid-Level) Systems
% Ogata, Modern Control Engineering, Ch. 4: Mathematical Modeling of
% Fluid Systems
%
% Consider a single tank of cross-sectional area $C$ with inflow rate
% $q_i(t)$ and outflow through a resistive valve to atmosphere, with
% head (liquid level) $h(t)$. Two quantities characterize the system:
%
% * *Capacitance* $C$: the change in stored liquid volume per unit
%   change in head, $C = dV/dh$ (here just the tank's cross-sectional
%   area for a tank with vertical sides).
% * *Resistance* $R$: relates the outflow rate to head via the
%   linearized relation $R = dh/dq_o$, valid for small perturbations
%   about an operating point.
%
% Conservation of mass (continuity) gives
%
% $C\frac{dh}{dt} = q_i - q_o, \qquad q_o = \frac{h}{R}$
%
% so
%
% $RC\dot{h} + h = Rq_i$

%% Transfer Function of the Liquid-Level System
% Taking the Laplace transform with zero initial conditions:
%
% $(RCs+1)H(s) = RQ_i(s) \quad\Rightarrow\quad
%   G(s) = \frac{H(s)}{Q_i(s)} = \frac{R}{RCs+1}$
%
% This is identical in form to the electrical RC circuit (single time
% constant $\tau=RC$), the fluid-electrical analogy:
%
%   Fluid                 Electrical
%   --------------------  --------------------
%   Flow rate, q           Current, i
%   Head, h                 Voltage, e
%   Resistance, R           Resistance, R
%   Capacitance, C          Capacitance, C
%
% With $R=2\,\mathrm{s/m^2}$ (valve resistance) and
% $C=5\,\mathrm{m^2}$ (tank area):
R = 2; C = 5;
G_tank = tf(R,[R*C 1])

figure
step(G_tank)
title('Liquid-Level Tank Step Response','Interpreter','latex','FontSize',20)
ylabel('$h(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% The time constant $\tau=RC=10\,\mathrm{s}$ is the time for the level
% to reach 63.2% of its final value -- the same first-order rule
% derived in |Transient and Steady-State/FirstOrder.m|.
tau = R*C;
disp(['Time constant tau = ' num2str(tau) ' s'])

%% Interacting Two-Tank System
% For two tanks in series, each with its own resistance and
% capacitance, the outflow of tank 1 feeds tank 2:
%
% $C_1\dot{h}_1 = q_i - \frac{h_1-h_2}{R_1}, \qquad
%   C_2\dot{h}_2 = \frac{h_1-h_2}{R_1} - \frac{h_2}{R_2}$
%
% giving the second-order transfer function
%
% $\frac{H_2(s)}{Q_i(s)} =
%   \frac{R_2}{R_1C_1R_2C_2 s^2 + (R_1C_1+R_2C_2+R_2C_1)s + 1}$
R1 = 1; C1 = 2; R2 = 2; C2 = 1;
num = R2;
den = [R1*C1*R2*C2, R1*C1+R2*C2+R2*C1, 1];
G_2tank = tf(num,den)

figure
step(G_2tank)
title('Two-Tank Interacting System Step Response','Interpreter','latex','FontSize',20)
ylabel('$h_2(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0,'HorizontalAlignment','right')
xlabel('$t$','Interpreter','latex','FontSize',20)
