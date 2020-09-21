function [dx] = ODEbicycleRollRigid2(t, y, u, C_beta, C_p, C_phi, C_r, E_beta, E_phi, E_p, E_r, D_beta, D_phi, D_p, D_r, C_delta, E_delta, D_delta, m, I_x, I_z, varargin)
% Rigid bicycle model with roll

% u(1) is deltal; u(2) is V_x
% x(1) is y_dot, x(2) is p_dot, x(3) is phi_dot, x(4) is r_dot


if abs(u(2)) <= 0.0001
    if u(2) >0
        u(2) = 0.0001;
    else
        u(2) = -0.0001;
    end
end

A = [ C_beta/(m*u(2)), C_p/m, C_phi/m, C_r/m-u(2);
    E_beta/(I_x*u(2)), E_p/I_x, E_phi/I_x, E_r/I_x;
    0, 1, 0, 0;
    D_beta/(I_z*u(2)), D_p/I_z, D_phi/I_z, D_r/I_z];

B = [C_delta/m;
    E_delta/I_x;
    0;
    D_delta/I_z];

dx = A*y + B*u(1);

end
%The roll-steering happens because of the suspension mechanisms that generate some steer angle when deflected.  