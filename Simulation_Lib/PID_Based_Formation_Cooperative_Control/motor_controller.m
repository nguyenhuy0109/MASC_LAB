function sigma = motor_controller(omega_des, P)
    sigma = (omega_des - P.omega_b) / P.C_R;
    sigma = min(max(sigma, P.sigma_min), P.sigma_max);

end
