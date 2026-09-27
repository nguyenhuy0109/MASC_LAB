% ===================================================
% SCRIPT KHAI BÁO THÔNG SỐ QUADROTOR (SABATINO THESIS)
% ===================================================
clear; clc;

% 1. Hằng số vật lý & Địa lý
g = 9.81;          % Gia tốc trọng trường (m/s^2)
m = 0.468;         % Khối lượng quadrotor (kg)
l = 0.225;         % Khoảng cách từ tâm quadrotor đến motor (m)

% 2. Momen quán tính các trục (kg.m^2)
Ix = 4.856e-3;     % Momen quán tính quanh trục x
Iy = 4.856e-3;     % Momen quán tính quanh trục y
Iz = 8.801e-3;     % Momen quán tính quanh trục z

% 3. Hệ số lực nâng & lực cản động học rotor
b = 2.980e-6;      % Thrust factor (N.s^2)
d = 1.140e-7;      % Drag factor (N.m.s^2)

% 4. Điểm cân bằng Lơ lửng (Hovering State)
ft_hover = m * g;  % Lực nâng tổng cân bằng với trọng lực (~4.591 N)

% ===================================================
% THIẾT LẬP LQR CHO QUADROTOR (SECTION 2.7 & 3.1)
% ===================================================

% Ma trận A (12x12) của mô hình tuyến tính hóa
A = zeros(12,12);
A(1,7) = 1;  A(2,8) = 1;  A(3,9) = 1;
A(4,10) = 1; A(5,11) = 1; A(6,12) = 1;
A(7,5) = -g; % ddot_x phu thuoc vao theta
A(8,4) = g;  % ddot_y phu thuoc vao phi

% Ma trận B (12x4)
B = zeros(12,4);
B(9,1)  = -1/m;   % u1 = ft anh huong den dz
B(10,2) = 1/Ix;   % u2 = tau_x anh huong den dphi
B(11,3) = 1/Iy;   % u3 = tau_y anh huong den dtheta
B(12,4) = 1/Iz;   % u4 = tau_z anh huong den dpsi

% Đặt trọng số Q mạnh hơn cho kênh Z và dz:
Q = diag([10, 10, 100, 5, 5, 5, 1, 1, 50, 1, 1, 1]); 
R = diag([0.1, 1, 1, 1]);

% Tính ma trận độ lợi phản hồi K (4x12)
K_lqr = lqr(A, B, Q, R);