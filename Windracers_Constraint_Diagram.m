%% Constraint diagram - single-motor UMX-class pylon racer (Windracers)
% AAE 451 method: wing loading (W/S) vs power loading (W/P), SI units.
% Constraints: stall, race speed ("cruise"), pylon turn (maneuver), climb.
% Anything marked ASSUMED is a placeholder - replace it with your own number.
clear; 
clc; 
close all;

%% 1) Mass budget [g]  ------------------------------------------------------
m_pcb       = 30;   % PCB/electronics (known) - mount aft to balance the motor
m_motor     = 5;    % single motor (known)
m_battery   = 6;   % ASSUMED
m_structure = 7;   % ASSUMED  rods + joints + wing + tail
m_misc      = 2;    % ASSUMED  prop, wires, etc.
m_total = (m_pcb + m_motor + m_battery + m_structure + m_misc)/1000;  % kg
g = 9.81;
W = m_total*g;      % N

%% 2) Requirements  ----------------------------------------------------------
rho       = 1.225;  % kg/m^3
Vs        = 2.0;    % m/s  stall speed (assumed your "2 seconds" means 2 m/s)
V_race    = 10;     % m/s  ASSUMED racing speed - set from the Purdue rules/course
thr_race  = 0.8;    % throttle margin at race speed (lecture uses ~0.76-0.8)
R_turn    = 6;      % m    ASSUMED pylon turn radius at V_race
thr_turn  = 1.0;    % full throttle in the turn
ROC       = 2;      % m/s  ASSUMED required climb rate
V_climb   = 1.3*Vs; % m/s  climb speed

%% 3) Aero/propulsion assumptions  -------------------------------------------
CLmax = 1.0;        % ASSUMED, low-Re (NACA 2412 at Re ~ 50k)
AR    = 5;          % ASSUMED
e     = 0.75;       % ASSUMED
CD0   = 0.04;       % ASSUMED zero-lift drag (exposed rods and joints hurt here)
eta_p = 0.5;        % ASSUMED propeller efficiency (tiny props are poor)
K     = 1/(pi*e*AR);

%% 4) Constraint equations  --------------------------------------------------
q = @(V) 0.5*rho*V.^2;

% Stall: vertical line (upper limit on W/S)
WS_stall = 0.5*rho*Vs^2*CLmax;

% Race speed, level flight, full drag polar (induced drag matters at this W/S)
qr = q(V_race);
f_cruise = @(ws) eta_p*thr_race ./ ( V_race*( qr*CD0./ws + K*ws/qr ) );

% Pylon turn: load factor from turn radius, then thrust to hold the turn
n = sqrt(1 + (V_race^2/(g*R_turn))^2);
f_man = @(ws) eta_p*thr_turn ./ ( V_race*( qr*CD0./ws + K*n^2*ws/qr ) );

% Climb (lecture form: climb near minimum-power speed), constant in W/S
LDmax   = 0.5*sqrt(pi*e*AR/CD0);
WP_clmb = eta_p / ( V_climb*( 1/(0.866*LDmax) + ROC/V_climb ) );
f_climb = @(ws) WP_clmb*ones(size(ws));

%% 5) Design point: max W/S (smallest wing), then max W/P (smallest motor)
WS_dp = WS_stall;
[WP_dp, k] = min([f_cruise(WS_dp), f_man(WS_dp), f_climb(WS_dp)]);
names = {'race speed','pylon turn','climb'};

S = W/WS_dp;  b = sqrt(AR*S);  c = S/b;
P_req = W/WP_dp;

%% 6) Plot  ------------------------------------------------------------------
WS = linspace(0.25*WS_stall, 2.5*WS_stall, 400);
figure('Color','w'); hold on; grid on;

ws_f  = WS(WS <= WS_stall);
upper = min([f_cruise(ws_f); f_man(ws_f); f_climb(ws_f)], [], 1);
fill([ws_f fliplr(ws_f)], [upper zeros(size(ws_f))], [0.75 0.9 0.75], ...
     'FaceAlpha',0.6,'EdgeColor','none','HandleVisibility','off');

plot(WS, f_cruise(WS), 'g-',  'LineWidth',2, 'DisplayName','Race speed');
plot(WS, f_man(WS),    'r-.', 'LineWidth',2, 'DisplayName','Pylon turn');
plot(WS, f_climb(WS),  'm-',  'LineWidth',2, 'DisplayName','Climb');
ymax = 1.3*WP_clmb;  ylim([0 ymax]);  xlim([WS(1) WS(end)]);
plot([WS_stall WS_stall], [0 ymax], 'b--', 'LineWidth',2, 'DisplayName','Stall speed');
plot(WS_dp, WP_dp, 'o', 'MarkerSize',10, 'MarkerFaceColor','y', ...
     'MarkerEdgeColor','k', 'DisplayName','Design point');
text(0.45*WS_stall, 0.35*WP_dp, 'Feasible', 'FontWeight','bold');

xlabel('Wing loading W/S  [N/m^2]');  ylabel('Power loading W/P  [N/W]');
title(sprintf('Constraint diagram  (W = %.0f g, V_s = %.1f m/s)', m_total*1000, Vs));
legend('Location','northeast');

%% 7) Results  ---------------------------------------------------------------
fprintf('Weight            : %.0f g (%.3f N)\n', m_total*1000, W);
fprintf('Design point      : W/S = %.2f N/m^2, W/P = %.3f N/W  (limited by %s)\n', ...
        WS_dp, WP_dp, names{k});
fprintf('Wing              : S = %.3f m^2, span = %.2f m, chord = %.2f m (AR %.1f)\n', S, b, c, AR);
fprintf('Installed power   : %.1f W  -> compare with your motor/battery limits\n', P_req);
fprintf('Turn load factor  : n = %.2f (bank %.0f deg), maneuver stall = %.1f m/s\n', ...
        n, acosd(1/n), Vs*sqrt(n));

fprintf('\nStall speed sweep (same mass, CLmax, AR):\n  Vs[m/s]  W/S    S[m^2]  span[m]  P[W]\n');
for Vtest = 2:5
    ws = 0.5*rho*Vtest^2*CLmax;
    wp = min([f_cruise(ws), f_man(ws), f_climb(ws)]);
    St = W/ws;
    fprintf('  %4.1f    %5.1f  %6.3f  %6.2f   %5.1f\n', Vtest, ws, St, sqrt(AR*St), W/wp);
end