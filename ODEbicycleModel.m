function [dx, y] = ODEbicycleModel(t, x, u, C_af, C_ar, m, l_f, l_r, I_z, varargin)
% u(1) is deltal; u(2) is V_x
% x(1) is y_dot, x(2) is psi_dot

y = x;

if abs(u(2)) <= 0.0001
    if u(2) >0
        u(2) = 0.0001;
    else
        u(2) = -0.0001;
    end
end

theta_vf = atan((x(1) + l_f*x(2))/u(2));
theta_vr = atan((x(1) - l_r*x(2))/u(2));

F_yf = C_af*(u(1) - theta_vf);
F_yr = -C_ar*theta_vr;

% dx = [(F_yf + F_yr)/m - u(2)*x(2);
%     (l_f*F_yf - l_r*F_yr)/I_z];

dx = [(F_yf*cos(u(1)) + F_yr)/m - u(2)*x(2);
    (l_f*F_yf*cos(u(1)) - l_r*F_yr)/I_z];

end

