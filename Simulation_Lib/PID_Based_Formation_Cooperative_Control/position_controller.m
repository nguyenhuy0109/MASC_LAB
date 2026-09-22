function [theta_d, phi_d, F_d, S] = position_controller(pos_des, vel_ff, pos, vel, psi, G, S, dt, P)
    % vong ngoai
    e_px = pos_des(1) - pos(1);
    e_py = pos_des(2) - pos(2);

    [vx_cmd, S.pos_x] = pid_update(G.pos_x, e_px, dt, S.pos_x, -P.max_vel_h, P.max_vel_h);
    [vy_cmd, S.pos_y] = pid_update(G.pos_y, e_py, dt, S.pos_y, -P.max_vel_h, P.max_vel_h);

    vx_des = vx_cmd + vel_ff(1);
    vy_des = vy_cmd + vel_ff(2);

    % vong trong
    e_vx = vx_des - vel(1);
    e_vy = vy_des - vel(2);

    a_max = P.g * tan(P.max_tilt);  
    [ax_des, S.vel_x] = pid_update(G.vel_x, e_vx, dt, S.vel_x, -a_max, a_max);
    [ay_des, S.vel_y] = pid_update(G.vel_y, e_vy, dt, S.vel_y, -a_max, a_max);

    %   ax = g*(cos(psi)*theta_d + sin(psi)*phi_d)
    %   ay = g*(sin(psi)*theta_d - cos(psi)*phi_d)
    cpsi = cos(psi); spsi = sin(psi);
    theta_d = ( cpsi*ax_des + spsi*ay_des ) / P.g;
    phi_d   = ( spsi*ax_des - cpsi*ay_des ) / P.g;

    theta_d = min(max(theta_d, -P.max_tilt), P.max_tilt);
    phi_d   = min(max(phi_d,   -P.max_tilt), P.max_tilt);

    %% 2 vong PID
    e_pz = pos_des(3) - pos(3);
    [vz_cmd, S.pos_z] = pid_update(G.pos_z, e_pz, dt, S.pos_z, -P.max_vel_z, P.max_vel_z);
    vz_des = vz_cmd + vel_ff(3);

    e_vz = vz_des - vel(3);
    az_lim = 1.5 * P.g; 
    [az_des, S.vel_z] = pid_update(G.vel_z, e_vz, dt, S.vel_z, -az_lim, az_lim);

    % Dynamics computation: (F_d/m)*cos(theta)*cos(phi) - g = az_des
    cc = cos(theta_d)*cos(phi_d);
    cc = max(cc, 0.5);  
    F_d = P.M * (az_des + P.g) / cc;
    F_d = min(max(F_d, P.F_min), P.F_max);

end
