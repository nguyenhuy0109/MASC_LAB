function S = position_controller_init()
    S.pos_x = pid_init();
    S.vel_x = pid_init();
    S.pos_y = pid_init();
    S.vel_y = pid_init();
    S.pos_z = pid_init();
    S.vel_z = pid_init();
end
