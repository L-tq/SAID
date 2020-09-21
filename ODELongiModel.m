function [dx, y] = ODELongiModel(t, x, u, C_thro, C_D, A, m, varargin)
% x(1) x_dot  vehicle speed

y = x;

vx = x(1);

throttle = u(1);
theta = u(2);
%% constants
g = 9.81;
pho = 1.206;

%% equation of motion

F_x = throttle*C_thro;

F_sum = F_x - m*g*sin(theta) - 1/2*pho*A*C_D*vx^2;

x_ddot = (F_sum)/m;

dx = x_ddot;

end