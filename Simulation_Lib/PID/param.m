clc; clear all;

m = 1.121;
g = 9.81;
Iv = diag([1e-2, 8.2e-3, 1.48e-2]);
kt = 5.11;
kd = 0.0487;
Lroll = 0.2136;
Lpitch = 0.1758;


% Kp_pos       = 0.83872;
% Ki_pos       = 0.0176027;
% Kd_pos       = 0.534976;
% Kp_att       = 8.80161;
% Ki_att       = 0.351368;
% Kd_att       = 2.46545;
% Kp_att_dot   = 2.14808;
% Ki_att_dot   = 0.132483;
% Kd_att_dot   = 0;

Kp_pos = [0.5 0.5 1]';
Kp_vel = [1.8 1.8 4]';
Ki_vel = [0.4 0.4 2.0]';
Kd_vel = [0.2 0.2 0]';

Kp_att = [6.5 6.5 0.4]';
Ki_att = [0 0 0]';
Kd_att = [0 0 0]';
Kp_att_dot = [0.053 0.055 0.09]';
Ki_att_dot = [0.1 0.1 0.1]';
Kd_att_dot = [0.0006 0.0004 0.0]';
