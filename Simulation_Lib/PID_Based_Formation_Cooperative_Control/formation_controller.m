function [pos_des, vel_ff] = formation_controller(leader_pos, leader_vel, leader_psi, rho, lambda, dz)
    ang = leader_psi + lambda;

    pos_des = [ leader_pos(1) + rho*cos(ang);
                leader_pos(2) + rho*sin(ang);
                leader_pos(3) + dz ];

    vel_ff = leader_vel;

end
