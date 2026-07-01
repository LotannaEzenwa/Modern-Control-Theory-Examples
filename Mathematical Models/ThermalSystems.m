%% Thermal Systems
% *Thermal resistance and capacitance: the thermometer as a first-order lag.*
%
% Ogata, _Modern Control Engineering_, Ch. 4.
%
% In this tutorial you will:
%
% * model a thermometer from a heat balance,
% * recognize the familiar first-order time constant $\tau=RC$, and
% * see how a thicker wall (larger R) slows the response.
%
% Step through with *Ctrl+Enter*, or render a report with |publish|.
%
% Thermal systems, like fluid systems, are characterized by a
% resistance and a capacitance:
%
% * *Thermal resistance* $R$: relates heat flow rate $q$ to temperature
%   difference $\Delta\theta$ across it, $R = \Delta\theta/q$.
% * *Thermal capacitance* $C$: relates stored heat to temperature,
%   $C = dQ/d\theta$ (mass times specific heat for a lumped body).
%
% Classic example: a mercury thermometer of capacitance $C$ immersed in
% a liquid bath at temperature $\theta_i(t)$, with thermal resistance
% $R$ between the bath and the thermometer's mercury (its glass wall).
% The thermometer's heat balance ($C\dot{\theta}_o$ = net heat flow in)
% gives
%
% $C\dot{\theta}_o = \frac{\theta_i - \theta_o}{R}
%   \quad\Rightarrow\quad RC\dot{\theta}_o + \theta_o = \theta_i$

%% Transfer Function of the Thermometer
% Taking the Laplace transform with zero initial conditions:
%
% $(RCs+1)\Theta_o(s) = \Theta_i(s) \quad\Rightarrow\quad
%   G(s) = \frac{\Theta_o(s)}{\Theta_i(s)} = \frac{1}{RCs+1}$
%
% This is again a single first-order lag, exactly analogous to the
% electrical RC circuit and the fluid tank: a thermometer plunged into
% a hot bath does not read the new temperature instantly, but
% approaches it exponentially with time constant $\tau=RC$.
%
% With $R=0.9\,^\circ\mathrm{C\cdot s/cal}$ and
% $C=0.1\,\mathrm{cal/^\circ C}$:
R = 0.9; C = 0.1;
tau = R*C;
G_thermo = tf(1,[tau 1])

figure
step(G_thermo)
title('Thermometer Step Response','Interpreter','latex','FontSize',20)
ylabel('$\theta_o(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%%
% At $t=\tau$, the response has reached $1-e^{-1}\approx 63.2\%$ of the
% final value -- consistent with the time-constant rule used for any
% first-order system in this repository.
[y,t] = step(G_thermo);
y_at_tau = interp1(t,y,tau);
disp(['Response at t = tau: ' num2str(y_at_tau) ' (expect ~0.632)'])

%% Effect of Thermometer Wall Thickness (Increasing R)
% A thicker glass wall increases $R$, slowing the thermometer's
% response without changing its final reading -- a direct illustration
% of how the time constant, not the DC gain, sets the speed of a
% first-order thermal system.
R_thick = 3;
G_thick = tf(1,[R_thick*C 1]);

figure
hold on
step(G_thermo)
step(G_thick)
hold off
legend('$R=0.9$','$R=3$ (thicker wall)','Interpreter','latex','FontSize',14)
title('Effect of Thermal Resistance on Response Speed','Interpreter','latex','FontSize',20)
ylabel('$\theta_o(t)$','Interpreter','latex','FontSize',20)
set(get(gca, 'YLabel'), 'Rotation', 0)
xlabel('$t$','Interpreter','latex','FontSize',20)

%% Try it yourself
% * Notice that changing |R| moves the time constant but never the final
%   reading -- try |R = 0.3| and confirm the steady value is unchanged.
% * Estimate the time to reach 95% (about 3*tau) and check it on the plot.
