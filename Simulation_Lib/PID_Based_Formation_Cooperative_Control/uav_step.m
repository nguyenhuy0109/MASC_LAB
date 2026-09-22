function [uav, log] = uav_step(uav, pos_des, vel_ff, psi_des, G, dt, P)
%   pos_des : [x_d;y_d;z_d] vi tri mong muon
%   vel_ff  : [vx;vy;vz] van toc feedforward
%   psi_des : goc yaw mong muon 
%   G       : struct he so PID
%   dt, P   : buoc thoi gian, tham so he thong
%
%   uav : struct UAV da cap nhat (trang thai moi)
%   log : struct chua cac gia tri trung gian (.theta_d, .phi_d, .F_d, .Tx, .Ty, .Tz)

    pos = uav.x(1:3); vel = uav.x(4:6);
    phi = uav.x(7); theta = uav.x(8); psi = uav.x(9);
    p = uav.x(10); q = uav.x(11); r = uav.x(12);

    [theta_d, phi_d, F_d, uav.Spos] = position_controller(pos_des, vel_ff, pos, vel, psi, G, uav.Spos, dt, P);
    [Tx, Ty, Tz, uav.Satt] = attitude_controller(theta_d, phi_d, psi_des, theta, phi, psi, p, q, r, G, uav.Satt, dt, P);

    omega_target = compute_omega_target(F_d, Tx, Ty, Tz, P);

    f = @(xx, uu) quad_full_dynamics(xx, uu, P);
    uav.x = rk4_step(f, uav.x, omega_target, dt);

    log.theta_d = theta_d; log.phi_d = phi_d; log.F_d = F_d;
    log.Tx = Tx; log.Ty = Ty; log.Tz = Tz;

end
