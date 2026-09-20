function omega_des = control_allocator(F_d, Tx, Ty, Tz, P)
    wt = P.MixInv * [F_d; Tx; Ty; Tz];   % [w1^2; w2^2; w3^2; w4^2]
    wt = max(wt, 0);

    omega_des = sqrt(wt);  
    omega_des = min(max(omega_des, P.omega_min), P.omega_max);

end
