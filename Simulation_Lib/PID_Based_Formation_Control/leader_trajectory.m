function [pos_d, vel_d, psi_d] = leader_trajectory(t)

    R        = 6.0;           
    omega_c  = 2*pi/20;        
    T_climb  = 8.0;             
    z_final  = 5.0;            
    T_yaw_ramp = 6.0;          

    if t <= 0
        theta = 0; thetadot = 0;
    elseif t < T_yaw_ramp
        u = t / T_yaw_ramp;
        % s(u) = 10u^3-15u^4+6u^5 
        S_u = 2.5*u^4 - 3*u^5 + u^6;
        s_u = 10*u^3 - 15*u^4 + 6*u^5;
        theta    = omega_c * T_yaw_ramp * S_u;
        thetadot = omega_c * s_u;
    else
        theta    = omega_c*T_yaw_ramp*0.5 + omega_c*(t - T_yaw_ramp);
        thetadot = omega_c;
    end

    ctheta = cos(theta); stheta = sin(theta);

    x = R*ctheta;
    y = R*stheta;
    xdot = -R*thetadot*stheta;
    ydot =  R*thetadot*ctheta;

    % minimum jerk 
    if t <= 0
        s = 0; sdot = 0;
    elseif t >= T_climb
        s = 1; sdot = 0;
    else
        tau = t / T_climb;
        s    = 10*tau^3 - 15*tau^4 + 6*tau^5;
        sdot = (30*tau^2 - 60*tau^3 + 30*tau^4) / T_climb;
    end

    z    = z_final * s;
    zdot = z_final * sdot;

    pos_d = [x; y; z];
    vel_d = [xdot; ydot; zdot];

    psi_d = theta + pi/2;

end
