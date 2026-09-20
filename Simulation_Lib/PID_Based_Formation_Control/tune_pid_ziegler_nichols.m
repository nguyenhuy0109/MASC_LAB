function G = tune_pid_ziegler_nichols()
% Cong thuc PID Ziegler-Nichols (tu Ku, Pu):
%   Kp = 0.6*Ku ,  Ti = Pu/2 ,  Td = Pu/8
%   Ki = Kp/Ti  ,  Kd = Kp*Td

    P = params();
    dt = P.dt;
    tau = P.tau_motor;   

    fprintf('=== Chinh dinh Ziegler-Nichols cho 12 vong PID ===\n');


    INNER_MARGIN = 0.5;

    G.rate_p = zn_tune_lag_integrator(P.Ixx, tau, dt, INNER_MARGIN);
    print_gain('rate_p (roll rate)', G.rate_p);

    G.rate_q = zn_tune_lag_integrator(P.Iyy, tau, dt, INNER_MARGIN);
    print_gain('rate_q (pitch rate)', G.rate_q);

    G.rate_r = zn_tune_lag_integrator(P.Izz, tau, dt, INNER_MARGIN);
    print_gain('rate_r (yaw rate)', G.rate_r);

   
    G.vel_z = zn_tune_lag_integrator(P.M, tau, dt, INNER_MARGIN);
    print_gain('vel_z (vertical speed)', G.vel_z);

    
    G.angle_phi = zn_tune_angle_loop(G.rate_p, P.Ixx, tau, dt, INNER_MARGIN);
    print_gain('angle_phi (roll angle)', G.angle_phi);

    G.angle_theta = zn_tune_angle_loop(G.rate_q, P.Iyy, tau, dt, INNER_MARGIN);
    print_gain('angle_theta (pitch angle)', G.angle_theta);

    G.angle_psi = zn_tune_angle_loop(G.rate_r, P.Izz, tau, dt, 1.0);
    print_gain('angle_psi (yaw angle)', G.angle_psi);

    
    G.pos_z = zn_tune_pos_loop_from_vel(G.vel_z, P.M, tau, dt, 1.0);
    print_gain('pos_z (altitude position)', G.pos_z);

    
    G.vel_x = zn_tune_horiz_vel_loop(G.angle_theta, G.rate_q, P.Iyy, tau, dt, P.g, INNER_MARGIN);
    print_gain('vel_x (horizontal speed x, via pitch)', G.vel_x);

    G.vel_y = zn_tune_horiz_vel_loop(G.angle_phi, G.rate_p, P.Ixx, tau, dt, P.g, INNER_MARGIN);
    print_gain('vel_y (horizontal speed y, via roll)', G.vel_y);

    
    POS_XY_MARGIN = 0.5;

    G.pos_x = zn_tune_pos_loop_generic(@(Kp) sim_pos_x_loop(Kp, G.vel_x, G.angle_theta, G.rate_q, P, dt), dt, 0.05, 2, POS_XY_MARGIN);
    print_gain('pos_x', G.pos_x);

    G.pos_y = zn_tune_pos_loop_generic(@(Kp) sim_pos_y_loop(Kp, G.vel_y, G.angle_phi, G.rate_p, P, dt), dt, 0.05, 2, POS_XY_MARGIN);
    print_gain('pos_y', G.pos_y);

    fprintf('=== Hoan tat chinh dinh ===\n');

    write_pid_gains_file(G);

end

function G = zn_tune_lag_integrator(I_or_M, tau, dt, margin)

    if nargin < 4, margin = 1.0; end
    N = 2000;
    sim_fn = @(Kp) sim_lag_integrator(Kp, I_or_M, tau, dt, N);
    [Ku, Pu] = find_ultimate_gain(sim_fn, dt, max(I_or_M/tau*0.02,1e-6), max(I_or_M/tau*2, 1e-3));
    G = zn_formula(Ku, Pu, margin);
end

function err = sim_lag_integrator(Kp, I_or_M, tau, dt, N)
    y = 0; uf = 0; y_des = 1.0;
    err = zeros(N,1);
    for k = 1:N
        e = y_des - y;
        u_cmd = Kp * e;   
        f = @(s,u) [ (u - s(1))/tau ; s(1)/I_or_M ];
        s = rk4_step(f, [uf; y], u_cmd, dt);
        uf = s(1); y = s(2);
        err(k) = e;
    end
end

function G = zn_tune_angle_loop(G_inner, I, tau, dt, margin)
    if nargin < 5, margin = 1.0; end
    N = 3000;
    sim_fn = @(Kp) sim_angle_loop(Kp, G_inner, I, tau, dt, N);
    [Ku, Pu] = find_ultimate_gain(sim_fn, dt, 0.2, 20);
    G = zn_formula(Ku, Pu, margin);
end

function err = sim_angle_loop(Kp_outer, G_inner, I, tau, dt, N)
    theta = 0; rate = 0; uf = 0; theta_des = 0.3;
    Sr = pid_init();
    err = zeros(N,1);
    for k = 1:N
        e_theta = theta_des - theta;
        rate_des = Kp_outer * e_theta;   % P-thuan cho vong ngoai
        e_rate = rate_des - rate;
        [u_cmd, Sr] = pid_update(G_inner, e_rate, dt, Sr, -1e6, 1e6);
        f = @(s,u) [ (u - s(1))/tau ; s(1)/I ; s(2) ];   % s=[uf; rate; theta]
        s = rk4_step(f, [uf; rate; theta], u_cmd, dt);
        uf = s(1); rate = s(2); theta = s(3);
        err(k) = e_theta;
    end
end

function G = zn_tune_pos_loop_from_vel(G_vel, M, tau, dt, margin)
    if nargin < 5, margin = 1.0; end
    N = 3000;
    sim_fn = @(Kp) sim_pos_from_vel(Kp, G_vel, M, tau, dt, N);
    [Ku, Pu] = find_ultimate_gain(sim_fn, dt, 0.2, 20);
    G = zn_formula(Ku, Pu, margin);
end

function err = sim_pos_from_vel(Kp_outer, G_vel, M, tau, dt, N)
    z = 0; vz = 0; uf = 0; z_des = 2.0;
    Sv = pid_init();
    err = zeros(N,1);
    for k = 1:N
        e_z = z_des - z;
        vz_des = Kp_outer * e_z;
        e_vz = vz_des - vz;
        [u_cmd, Sv] = pid_update(G_vel, e_vz, dt, Sv, -1e6, 1e6);
        f = @(s,u) [ (u - s(1))/tau ; s(1)/M ; s(2) ];
        s = rk4_step(f, [uf; vz; z], u_cmd, dt);
        uf = s(1); vz = s(2); z = s(3);
        err(k) = e_z;
    end
end

function G = zn_tune_horiz_vel_loop(G_angle, G_rate, I, tau, dt, g, margin)
    if nargin < 7, margin = 1.0; end
    N = 4000;
    sim_fn = @(Kp) sim_horiz_vel_loop(Kp, G_angle, G_rate, I, tau, dt, g, N);
    [Ku, Pu] = find_ultimate_gain(sim_fn, dt, 0.2, 20);
    G = zn_formula(Ku, Pu, margin);
end

function err = sim_horiz_vel_loop(Kp_outer, G_angle, G_rate, I, tau, dt, g, N)
    vx = 0; theta = 0; rate = 0; uf = 0; vx_des = 1.0;
    Sa = pid_init(); Sr = pid_init();
    err = zeros(N,1);
    for k = 1:N
        e_vx = vx_des - vx;
        a_des = Kp_outer * e_vx;                 % P-thuan cho vong ngoai (dang chinh dinh)
        theta_des = a_des / g;

        e_theta = theta_des - theta;
        [rate_des, Sa] = pid_update(G_angle, e_theta, dt, Sa, -1e6, 1e6);
        e_rate = rate_des - rate;
        [u_cmd, Sr] = pid_update(G_rate, e_rate, dt, Sr, -1e6, 1e6);

        f = @(s,u) [ (u - s(1))/tau ; s(1)/I ; s(2) ];   
        s = rk4_step(f, [uf; rate; theta], u_cmd, dt);
        uf = s(1); rate = s(2); theta = s(3);

        a_actual = g * theta;
        vx = vx + dt * a_actual;  

        err(k) = e_vx;
    end
end

function G = zn_tune_pos_loop_generic(sim_fn, dt, Kp_lo, Kp_hi, margin)
    if nargin < 5, margin = 1.0; end
    [Ku, Pu] = find_ultimate_gain(sim_fn, dt, Kp_lo, Kp_hi);
    G = zn_formula(Ku, Pu, margin);
end

function err = sim_pos_x_loop(Kp_outer, G_vx, G_angle, G_rate, P, dt)
    N = 4000; g = P.g; I = P.Iyy; tau = P.tau_motor;
    x = 0; vx = 0; theta = 0; rate = 0; uf = 0; x_des = 2.0;
    Sv = pid_init(); Sa = pid_init(); Sr = pid_init();
    err = zeros(N,1);
    for k = 1:N
        e_x = x_des - x;
        vx_des = Kp_outer * e_x;
        e_vx = vx_des - vx;
        [a_des, Sv] = pid_update(G_vx, e_vx, dt, Sv, -1e6, 1e6);
        theta_des = a_des / g;

        e_theta = theta_des - theta;
        [rate_des, Sa] = pid_update(G_angle, e_theta, dt, Sa, -1e6, 1e6);
        e_rate = rate_des - rate;
        [u_cmd, Sr] = pid_update(G_rate, e_rate, dt, Sr, -1e6, 1e6);

        f = @(s,u) [ (u - s(1))/tau ; s(1)/I ; s(2) ];
        s = rk4_step(f, [uf; rate; theta], u_cmd, dt);
        uf = s(1); rate = s(2); theta = s(3);

        a_actual = g*theta;
        vx = vx + dt*a_actual;
        x  = x  + dt*vx;

        err(k) = e_x;
    end
end

function err = sim_pos_y_loop(Kp_outer, G_vy, G_angle, G_rate, P, dt)
    N = 4000; g = P.g; I = P.Ixx; tau = P.tau_motor;
    y = 0; vy = 0; phi = 0; rate = 0; uf = 0; y_des = 2.0;
    Sv = pid_init(); Sa = pid_init(); Sr = pid_init();
    err = zeros(N,1);
    for k = 1:N
        e_y = y_des - y;
        vy_des = Kp_outer * e_y;
        e_vy = vy_des - vy;
        [a_des, Sv] = pid_update(G_vy, e_vy, dt, Sv, -1e6, 1e6);
        phi_des = a_des / g;

        e_phi = phi_des - phi;
        [rate_des, Sa] = pid_update(G_angle, e_phi, dt, Sa, -1e6, 1e6);
        e_rate = rate_des - rate;
        [u_cmd, Sr] = pid_update(G_rate, e_rate, dt, Sr, -1e6, 1e6);

        f = @(s,u) [ (u - s(1))/tau ; s(1)/I ; s(2) ];
        s = rk4_step(f, [uf; rate; phi], u_cmd, dt);
        uf = s(1); rate = s(2); phi = s(3);

        a_actual = g*phi;
        vy = vy + dt*a_actual;
        y  = y  + dt*vy;

        err(k) = e_y;
    end
end


function G = zn_formula(Ku, Pu, margin)
    if nargin < 3, margin = 1.0; end
    KI_DERATE = 0.05;

    Kp = 0.6 * Ku * margin;
    Ti = Pu / 2;
    Td = Pu / 8;
    Ki = (Kp / Ti) * KI_DERATE;
    Kd = Kp * Td;
    G = [Kp, Ki, Kd];
end

function print_gain(name, G)
    fprintf('  %-28s Kp=%10.5f  Ki=%10.5f  Kd=%10.5f\n', name, G(1), G(2), G(3));
end

function write_pid_gains_file(G)
    fid = fopen('pid_gains.m', 'w');
    fprintf(fid, 'function G = pid_gains()\n');
    fprintf(fid, '%% PID_GAINS  He so PID cho 12 vong dieu khien, tao tu dong boi\n');
    fprintf(fid, '%% tune_pid_ziegler_nichols.m (phuong phap Ziegler-Nichols).\n');
    fn = fieldnames(G);
    for i = 1:numel(fn)
        v = G.(fn{i});
        fprintf(fid, 'G.%s = [%.8f, %.8f, %.8f];\n', fn{i}, v(1), v(2), v(3));
    end
    fprintf(fid, '\nend\n');
    fclose(fid);
    fprintf('Da ghi he so PID vao pid_gains.m\n');
end
