clc; clear all;

m = 1.121;
g = 9.81;
Iv = diag([1e-2, 8.2e-3, 1.48e-2]);
kt = 5.11;
kd = 0.0487;
Lroll = 0.2136;
Lpitch = 0.1758;

Kp_pos       = 2.53872;
Ki_pos       = 0.0176027;
Kd_pos       = 0.534976;
Kp_att       = 8.80161;
Ki_att       = 0.351368;
Kd_att       = 2.46545;
Kp_att_dot   = 0.14808;
Ki_att_dot   = 0.132483;
Kd_att_dot   = 0;