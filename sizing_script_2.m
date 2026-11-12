% Sizing script 


clear; clc; close all;

%% 1) Constants, aero assumptions, requirements (from the team diagram)
rho = 1.225;  g = 9.81;
CD0 = 0.045;  e = 0.75;  AR = 4.52;  CL_max = 1.35;  alpha_stall = 15;
V_stall = 2.0;  V_max = 6.0;  ROC = 1.0;  V_climb = 3.0;  S_TO = 3.0;
mu = 0.05;  CL_TO = 0.8;
CD_TO = CD0 + CL_TO^2/(pi*e*AR);
V_turn = V_max;  R_turn = 6;          
K = 1/(pi*e*AR);

%% 2) Constraint curves: required T/W at each wing loading W/S [N/m^2]
WS = linspace(1, 16, 1000);

q_s = 0.5*rho*V_stall^2;                                   % stall (thrust-assisted)
TW_stall = max((1 - q_s*CL_max./WS)/sind(alpha_stall), 0);

q_m = 0.5*rho*V_max^2;                                     % max speed
TW_maxspeed = q_m*CD0./WS + K*WS/q_m;

gam = asin(ROC/V_climb);  q_c = 0.5*rho*V_climb^2;         % climb (exact angle)
TW_roc = q_c*CD0./WS + K*cos(gam)^2*WS/q_c + sin(gam);

TW_takeoff = mu + 0.72*(CD_TO - mu*CL_TO)/CL_max + 1.44*WS/(g*rho*CL_max*S_TO); % takeoff ground roll
             

n = sqrt(1 + (V_turn^2/(g*R_turn))^2);  q_t = 0.5*rho*V_turn^2;   % pylon turn
TW_turn = q_t*CD0./WS + K*n^2*WS/q_t;

%% 3) Feasible region and design point
% The design must meet the HIGHEST T/W of all constraints at each W/S.
TW_required = max([TW_stall; TW_maxspeed; TW_roc; TW_takeoff; TW_turn], [], 1);
[TW_design, idx] = min(TW_required);   % lowest T/W = smallest motor
WS_design = WS(idx);                   % auto-picked from the diagram
% EDIT: to force a wing loading instead, uncomment these two lines:
% WS_design = 4.0;  TW_design = interp1(WS, TW_required, WS_design);

%% 4) Component weights [g]
m_motor_prop = 3;   m_front_gear = 2;   m_back_gear = 4;   % back gear: 2 g for BOTH (EDIT if each)
m_bracket = 2;   
m_tail = 1; 
m_boom = 1;
m_pcb = 6;   
m_battery = 4;
m_fixed = m_motor_prop + m_front_gear + m_back_gear + m_bracket + m_tail + m_boom + m_pcb + m_battery; % everything except the wing

% Wing: area-density = wing mass / wing area
% current design
m_wing_now = 12;   
S_now = 0.0724638;

rho_A_now = m_wing_now/S_now; % g/m^2
rho_A = rho_A_now; % EDIT: g/m^2, set lower if you expect a lighter wing (e.g. 100)

% Comparison Aircraft Specifications
% Night Vapor
m_nightvapor = 26;                % total mass [g]
b_nightvapor = 0.37;              % wingspan [m] (37 cm)
c_nightvapor = 0.15;              % chord [m] (15 cm)
S_nightvapor = b_nightvapor * c_nightvapor; % wing area [m^2] = 0.0555 m^2
WS_nightvapor = (m_nightvapor / 1000 * g) / S_nightvapor; % W/S [N/m^2]

% Sports Cub
m_sportscub = 56;                 % total mass [g]
b_sportscub = 0.61;               % wingspan [m] (61 cm)
c_sportscub = 0.10;               % chord [m] (10 cm)
S_sportscub = b_sportscub * c_sportscub;   % wing area [m^2] = 0.061 m^2
WS_sportscub = (m_sportscub / 1000 * g) / S_sportscub;   % W/S [N/m^2]

%% 5) Sizing loop: iterate until guess = calculated
wing_wt_per_area = rho_A/1000*g; % N/m^2 of wing area
if wing_wt_per_area >= WS_design
    error('Wing alone weighs %.2f N/m^2, more than W/S = %.2f. No solution: lower rho_A or raise W/S.', wing_wt_per_area, WS_design);
end

% initial guess [g]
m_guess = 40;   
tol = 1e-6;   
maxIter = 100;               
fprintf('Iter  guess[g]  S[m^2]   wing[g]  calc[g]\n');

for k = 1:maxIter
    W_guess = m_guess/1000*g;            % N
    S       = W_guess/WS_design;         % wing area from the guess
    m_wing  = rho_A*S;                   % wing mass from area-density
    m_calc  = m_fixed + m_wing;          % total from the parts
    fprintf('%3d   %7.3f  %7.4f  %7.3f  %7.3f\n', k, m_guess, S, m_wing, m_calc);
    if abs(m_calc - m_guess) < tol 
        break; 
    end
    m_guess = m_calc;                    % feed the result back in
end
m_total = m_calc;   
W = m_total/1000*g;   
S = W/WS_design;

% Closed-form check: m = m_fixed/(1 - rho_A*g/(1000*W/S)) should match the loop
m_check = m_fixed/(1 - wing_wt_per_area/WS_design);

%% 6) Results
b = sqrt(AR*S);  
c = S/b;
Vs_aero = sqrt(2*WS_design/(rho*CL_max));                  % stall with NO thrust help
fprintf('\nDesign point      : W/S = %.2f N/m^2, T/W = %.2f\n', WS_design, TW_design);
fprintf('Total mass        : %.1f g  (loop) vs %.1f g (closed form)\n', m_total, m_check);
fprintf('Wing              : S = %.4f m^2, span = %.2f m, chord = %.3f m, wing = %.1f g\n', S, b, c, rho_A*S);
fprintf('Thrust needed     : %.1f gf  (T/W x weight)\n', TW_design*m_total);
fprintf('Stall (no thrust) : %.2f m/s  (target %.1f m/s)\n', Vs_aero, V_stall);
fprintf('Current design    : %.1f g, W/S = %.2f N/m^2\n', m_fixed + m_wing_now, (m_fixed + m_wing_now)/1000*g/S_now);

%% 7) Sensitivity: how the answer moves with wing area-density
fprintf('\nrho_A[g/m^2]  mass[g]  S[m^2]  span[m]\n');
for rA = [60 80 100 120 140 rho_A_now]
    wpa = rA/1000*g;
    if wpa >= WS_design 
        continue; 
    end
    mm = m_fixed/(1 - wpa/WS_design);   SS = mm/1000*g/WS_design;
    fprintf('  %6.1f      %6.1f   %.4f   %.2f\n', rA, mm, SS, sqrt(AR*SS));
end

%% 8) Plot
figure('Color','w'); hold on; grid on;
fill([WS fliplr(WS)], [TW_required 1.5*ones(size(WS))], [0.8 0.95 0.8],'EdgeColor','none','HandleVisibility','off');
plot(WS, TW_maxspeed, 'b-',  'LineWidth',2, 'DisplayName','Max speed');
plot(WS, TW_roc,      'r--', 'LineWidth',2, 'DisplayName','Climb');
plot(WS, TW_takeoff,  'k-.', 'LineWidth',2, 'DisplayName','Takeoff');
plot(WS, TW_stall,    'm-',  'LineWidth',2, 'DisplayName','Stall');
plot(WS, TW_turn,     'c-',  'LineWidth',2, 'DisplayName','Pylon turn');

plot(WS_design, TW_design, 'p', 'MarkerSize',14, 'MarkerFaceColor','y', 'MarkerEdgeColor','k', 'DisplayName','Design point');
WS_now = (m_fixed + m_wing_now)/1000*g/S_now;
plot([WS_now WS_now], [0 1.5], 'k:', 'LineWidth',1.5, 'DisplayName','Current design W/S');

% Comparison aircraft
plot([WS_nightvapor WS_nightvapor], [0 1.5], 'g--', 'LineWidth',1.5, 'DisplayName', sprintf('Night Vapor (W/S = %.2f)', WS_nightvapor));
plot([WS_sportscub WS_sportscub], [0 1.5], 'm--', 'LineWidth',1.5, 'DisplayName', sprintf('Sports Cub (W/S = %.2f)', WS_sportscub));

axis([1 16 0 1.5]);
xlabel('Wing loading W/S [N/m^2]');  
ylabel('Thrust-to-weight T/W');
title(sprintf('Constraint diagram, sized mass = %.1f g', m_total));
legend('Location','northeast');