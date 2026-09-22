function uav = uav_init(P, x0)
    if nargin < 2 || isempty(x0)
        x0 = zeros(16,1);
        x0(13:16) = P.omega_hover;
    end

    uav.x    = x0;
    uav.Spos = position_controller_init();
    uav.Satt = attitude_controller_init();

end
