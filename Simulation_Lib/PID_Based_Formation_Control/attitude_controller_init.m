function S = attitude_controller_init()
    S.angle_phi   = pid_init();
    S.rate_p      = pid_init();
    S.angle_theta = pid_init();
    S.rate_q      = pid_init();
    S.angle_psi   = pid_init();
    S.rate_r      = pid_init();
end
