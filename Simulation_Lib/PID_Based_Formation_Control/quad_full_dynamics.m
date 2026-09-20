function xdot = quad_full_dynamics(x, omega_target, P)
% Trang thai x (16x1):
%   x(1:12)  : nhu quad_dynamics.m (vi tri, van toc, goc Euler, toc do goc)
%   x(13:16) : [w1;w2;w3;w4]  toc do goc thuc te cua 4 dong co (rad/s)
%
% Dau vao omega_target (4x1): toc do dong co target
%
%   omega_i_dot = (omega_target_i - w_i) / tau_motor
%   [F;Tx;Ty;Tz] = Mix * w.^2   

    w = x(13:16);
    wdot = (omega_target - w) / P.tau_motor;

    Fvec = P.Mix * (w.^2);   % [F; Tx; Ty; Tz] thuc te tac dung len UAV

    rigid_xdot = quad_dynamics(x(1:12), Fvec, P);

    xdot = [rigid_xdot; wdot];

end
