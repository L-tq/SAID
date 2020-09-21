function [dx, y] = ODEyawRoll(t, x, u, m, l_f, l_r, h, C_af, C_ar, C_phi, K_phi, I_x, I_y, I_z, varargin)
% Rigid bicycle model with roll



y = x;

if abs(u(2)) <= 0.0001
    if u(2) >0
        u(2) = 0.0001;
    else
        u(2) = -0.0001;
    end
end
%% control inputs
delta = u(1);
% PwrEngO = u(2)*1000; % represents throttle for now
vx = u(2);
vx_dot = u(3);

%% states 
% vx = x(1);
vy = x(1);
phi_dot = x(2);
phi = x(3);
psi_dot = x(4);

%% Fx
% kineticE = 1/2*m*vx^2; % Kinetic energy in x direction
FxT = vx_dot*m;

%% Fy
theta_vf = atan((vy + l_f*psi_dot)/vx);
theta_vr = atan((vy - l_r*psi_dot)/vx);

alpha_f = delta - theta_vf;
alpha_r = -theta_vr;

F_yf = C_af*alpha_f;
F_yr = -C_ar*alpha_r;

%% total Fy
FyT = F_yf + F_yr;
%% Moment
Mt = l_f*F_yf*cos(u(1)) - l_r*F_yr;

%% coupling between the yaw and roll accelerations
% vy_dot = (FyT - m * psi_dot * vx + m * h * sin(phi) * psi_dot^2)*...
%     (I_x + m*h^2)/(m*I_x) + h * (m * h * psi_dot * vx + C_phi * phi + ...
%     K_phi * phi_dot - (m*h^2 + I_y - I_z) * psi_dot^2 * sin(phi))/I_x;

% psi_ddot = (Mt * + FxT * sin(phi) * h + 2*m*h^2* sin(phi) * phi_dot * psi_dot)/...
%     (I_z - h^2*phi^2*m);
% 
% phi_ddot = (-FyT*h - (C_phi + (I_z - I_y) * psi_dot^2) * phi - K_phi*phi_dot)/I_x;

%%
% vy_dot = (FyT - m * psi_dot * vx + m * h * phi * psi_dot^2)*...
%     (I_x + m*h^2)/(m*I_x) + h * (m * h * psi_dot * vx + C_phi * phi + ...
%     K_phi * phi_dot - (m*h^2 + I_y - I_z) * psi_dot^2 * phi)/I_x;

% psi_ddot = (Mt * + FxT * phi * h + 2*m*h^2* phi * phi_dot * psi_dot)/...
%     (I_z - h^2*phi^2*m);

vy_dot = (F_yf*cos(delta) + F_yr)/m - vx*psi_dot;
psi_ddot = (l_f*F_yf*cos(delta) - l_r*F_yr)/I_z;

phi_ddot = (-FyT*h - (C_phi + (I_z - I_y) * psi_dot^2) * phi - K_phi*phi_dot)/I_x;
%%
% dx = [vx_dot;vy_dot; phi_ddot; phi_dot; psi_ddot;];
dx = [vy_dot; phi_ddot; phi_dot; psi_ddot];

end

%% literatures
% A New Predictive Lateral Load Transfer Ratio for Rollover Prevention Systems

% Vehicle Dynamics Control and Controller Allocation for Rollover Prevention