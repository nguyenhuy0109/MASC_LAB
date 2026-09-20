function P = params()

P.M_b = 0.267;              % khoi luong pin (kg)
P.M_d = 0.854;               % khoi luong than UAV (kg)
P.M   = P.M_b + P.M_d;        % tong khoi luong (kg)

P.lin_hover_trim = 0.61698;   % lenh dong co (throttle, 0..1) tai hover
P.g   = 9.81;                 % gia toc trong truong (m/s^2)

P.L_roll  = 0.2136;           % khoang cach dong co - dong co theo truc roll (m)
P.L_pitch = 0.1758;           % khoang cach dong co - dong co theo truc pitch (m)

P.Cobra_2208_Kv = 1300;       % KV dong co Cobra 2208 (rpm/V)

P.kt = 60/(2*pi*P.Cobra_2208_Kv);  % hang so mo-men dong co (Nm/A)
P.kv = P.Cobra_2208_Kv*(2*pi/60);  % hang so toc do dong co (rad/s/V)

P.Ahover = 5.7856;            % dong dien dong co toi da tai 100% throttle (A)
P.Jxx = 0.0100000;            % kg*m^2
P.Jyy = 0.0082000;            % kg*m^2
P.Jzz = 0.0148000;            % kg*m^2
P.Vd  = 12.6;                 % dien ap pin toi da (V)

P.Ixx = P.Jxx;
P.Iyy = P.Jyy;
P.Izz = P.Jzz;

P.k1 = 0.05;   % he so can tinh tien theo x (kg/s)
P.k2 = 0.05;   % he so can tinh tien theo y (kg/s)
P.k3 = 0.08;   % he so can tinh tien theo z (kg/s) 


P.V_hover = P.lin_hover_trim * P.Vd;

% omega = C_R*sigma + omega_b
% omega_b = 0 omega = kv*(sigma*Vd) => C_R = kv*Vd.
P.omega_b = 0;
P.C_R = P.kv * P.Vd;                 % (rad/s) per (throttle unit)

% Toc do dong co tai hover
P.omega_hover = P.C_R * P.lin_hover_trim + P.omega_b;

%   4*C_T*omega_hover^2 = M*g 
P.Thrust_hover_per_motor = P.M * P.g / 4;
P.C_T = P.Thrust_hover_per_motor / (P.omega_hover^2);

P.I_hover = P.lin_hover_trim * P.Ahover;
P.tau_hover = P.kt * P.I_hover;
P.C_M = P.tau_hover / (P.omega_hover^2);


a_r = P.L_roll  / 2;   
a_p = P.L_pitch / 2;   

% [F; Tx; Ty; Tz] = Mix * [w1^2; w2^2; w3^2; w4^2]
P.Mix = [  P.C_T,        P.C_T,        P.C_T,        P.C_T      ;
           a_r*P.C_T,    a_r*P.C_T,   -a_r*P.C_T,   -a_r*P.C_T   ;
           a_p*P.C_T,   -a_p*P.C_T,   -a_p*P.C_T,    a_p*P.C_T   ;
           P.C_M,       -P.C_M,        P.C_M,       -P.C_M      ];

P.MixInv = inv(P.Mix);


P.sigma_min = 0;
P.sigma_max = 1;
P.omega_max = P.C_R * P.sigma_max + P.omega_b;
P.omega_min = 0;


P.tau_motor = 0.04;  


P.max_tilt     = deg2rad(25);   % goc nghieng toi da (roll/pitch), rad
P.max_vel_h    = 5;             % toc do ngang toi da lam setpoint trung gian (m/s)
P.max_vel_z    = 3;             % toc do len/xuong toi da (m/s)
P.F_min        = 0;             % luc day tong toi thieu (N)
P.F_max        = 4 * P.C_T * P.omega_max^2;  % luc day tong toi da (N)
P.T_max        = 1.5;           % gioi han mo-men Tx,Ty,Tz trung gian (N.m) 


P.dt   = 0.01;  
P.Tsim = 30;    

end
