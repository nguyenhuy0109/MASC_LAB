function [Tx, Ty, Tz, S] = attitude_controller(theta_d, phi_d, psi_d, theta, phi, psi, p, q, r, G, S, dt, P)

    e_phi = phi_d - phi;
    [p_des, S.angle_phi] = pid_update(G.angle_phi, e_phi, dt, S.angle_phi, -8, 8);
    e_p = p_des - p;
    [Tx, S.rate_p] = pid_update(G.rate_p, e_p, dt, S.rate_p, -P.T_max, P.T_max);

    e_theta = theta_d - theta;
    [q_des, S.angle_theta] = pid_update(G.angle_theta, e_theta, dt, S.angle_theta, -8, 8);
    e_q = q_des - q;
    [Ty, S.rate_q] = pid_update(G.rate_q, e_q, dt, S.rate_q, -P.T_max, P.T_max);

    e_psi = wrap_to_pi(psi_d - psi);
    [r_des, S.angle_psi] = pid_update(G.angle_psi, e_psi, dt, S.angle_psi, -4, 4);
    e_r = r_des - r;
    [Tz, S.rate_r] = pid_update(G.rate_r, e_r, dt, S.rate_r, -P.T_max, P.T_max);

end
