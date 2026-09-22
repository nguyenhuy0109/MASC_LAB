function omega_target = compute_omega_target(F_d, Tx, Ty, Tz, P)
    omega_des = control_allocator(F_d, Tx, Ty, Tz, P);
    sigma     = motor_controller(omega_des, P);
    omega_target = P.C_R * sigma + P.omega_b;

end
