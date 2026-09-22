clear; close all; clc;

P = params();
G = pid_gains();
dt = P.dt;
Tsim = P.Tsim;   % 30 (s)
N = round(Tsim/dt);
tvec = (0:N-1)*dt;



rho    = 2.0;                 % khoang cach mong muon toi leader (m)
lambda1 = deg2rad(150);        % goc lech follower 1 
lambda2 = -deg2rad(150);       % goc lech follower 2
dz     = 0;                    % cung do cao voi leader


[pos_d0, vel_d0, psi_d0] = leader_trajectory(0);

x0_leader = zeros(16,1);
x0_leader(1:3) = pos_d0;
x0_leader(9)   = psi_d0;
x0_leader(13:16) = P.omega_hover;

[posF1_0, ~] = formation_controller(pos_d0, vel_d0, psi_d0, rho, lambda1, dz);
[posF2_0, ~] = formation_controller(pos_d0, vel_d0, psi_d0, rho, lambda2, dz);

x0_f1 = zeros(16,1); x0_f1(1:3) = posF1_0; x0_f1(9) = psi_d0; x0_f1(13:16) = P.omega_hover;
x0_f2 = zeros(16,1); x0_f2(1:3) = posF2_0; x0_f2(9) = psi_d0; x0_f2(13:16) = P.omega_hover;

leader = uav_init(P, x0_leader);
follower1 = uav_init(P, x0_f1);
follower2 = uav_init(P, x0_f2);


LEADER_TRAJ = zeros(N,3);
F1_TRAJ     = zeros(N,3);
F2_TRAJ     = zeros(N,3);
LEADER_DES  = zeros(N,3);
F1_DES      = zeros(N,3);
F2_DES      = zeros(N,3);

fprintf('Dang mo phong %.0f giay (%d buoc, dt=%.3fs)...\n', Tsim, N, dt);


for k = 1:N
    t = tvec(k);

    % --- Quy dao leader ---
    [pos_d_L, vel_d_L, psi_d_L] = leader_trajectory(t);
    [leader, ~] = uav_step(leader, pos_d_L, vel_d_L, psi_d_L, G, dt, P);

    % --- Doi hinh cho 2 follower, dua tren trang thai cua leader ---
    leader_pos_actual = leader.x(1:3);
    leader_vel_actual = leader.x(4:6);
    leader_psi_actual = leader.x(9);

    [pos_d_F1, vel_ff_F1] = formation_controller(leader_pos_actual, leader_vel_actual, leader_psi_actual, rho, lambda1, dz);
    [follower1, ~] = uav_step(follower1, pos_d_F1, vel_ff_F1, leader_psi_actual, G, dt, P);

    [pos_d_F2, vel_ff_F2] = formation_controller(leader_pos_actual, leader_vel_actual, leader_psi_actual, rho, lambda2, dz);
    [follower2, ~] = uav_step(follower2, pos_d_F2, vel_ff_F2, leader_psi_actual, G, dt, P);

  
    LEADER_TRAJ(k,:) = leader.x(1:3)';
    F1_TRAJ(k,:)      = follower1.x(1:3)';
    F2_TRAJ(k,:)      = follower2.x(1:3)';
    LEADER_DES(k,:)   = pos_d_L';
    F1_DES(k,:)       = pos_d_F1';
    F2_DES(k,:)       = pos_d_F2';

    if mod(k, round(N/10)) == 0
        fprintf('  ... %3.0f%%\n', 100*k/N);
    end
end

fprintf('Hoan tat mo phong.\n');

%% ---- Luu ket qua ----
save('formation_sim_results.mat', 'tvec', 'LEADER_TRAJ', 'F1_TRAJ', 'F2_TRAJ', ...
     'LEADER_DES', 'F1_DES', 'F2_DES', 'P', 'G');

%% ---- Thong ke sai so bam quy dao ----
err_L  = sqrt(sum((LEADER_TRAJ - LEADER_DES).^2, 2));
err_F1 = sqrt(sum((F1_TRAJ - F1_DES).^2, 2));
err_F2 = sqrt(sum((F2_TRAJ - F2_DES).^2, 2));

fprintf('\n--- Thong ke sai so bam quy dao (m) ---\n');
fprintf('Leader:     TB=%.3f  Max=%.3f\n', mean(err_L),  max(err_L));
fprintf('Follower1:  TB=%.3f  Max=%.3f\n', mean(err_F1), max(err_F1));
fprintf('Follower2:  TB=%.3f  Max=%.3f\n', mean(err_F2), max(err_F2));

%% ---- Ve bieu do ----
plot_results('formation_sim_results.mat');
