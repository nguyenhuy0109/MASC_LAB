function xdot = quad_dynamics(x, u, P)
% Trang thai x (12x1):
%   x(1:3)  = [X, Y, Z]           vi tri trong he toa do mat dat
%   x(4:6)  = [Xdot, Ydot, Zdot]  van toc tinh tien
%   x(7:9)  = [phi, theta, psi]   goc Euler (roll, pitch, yaw)
%   x(10:12)= [phidot, thetadot, psidot]  van toc goc (dao ham Euler)
%
% Dau vao dieu khien u (4x1):
%   u(1) = u1 = F   (tong luc day, N)
%   u(2) = u2 = Tx  (mo-men roll, N.m)
%   u(3) = u3 = Ty  (mo-men pitch, N.m)
%   u(4) = u4 = Tz  (mo-men yaw, N.m)
%
% P : struct tham so tu params.m (can P.M, P.g, P.k1,k2,k3, P.Ixx/Iyy/Izz)

    X = x(1); Y = x(2); Z = x(3); 
    Xd = x(4); Yd = x(5); Zd = x(6);
    phi = x(7); theta = x(8); psi = x(9);
    phid = x(10); thetad = x(11); psid = x(12);

    u1 = u(1); u2 = u(2); u3 = u(3); u4 = u(4);

    if isfield(P, 'd') && ~isempty(P.d)
        d = P.d;
    else
        d = zeros(6,1);
    end

    sphi = sin(phi);   cphi = cos(phi);
    sth  = sin(theta); cth  = cos(theta);
    spsi = sin(psi);   cpsi = cos(psi);

    m = P.M;

    % --- Phuong trinh vi tri ---
    %   Ydd = (u1/m)(spsi*sth*cphi - cpsi*sphi) - (k2/m)*Ydot + d2
    Xdd = (u1/m)*(spsi*sphi + cpsi*cphi*sth) - (P.k1/m)*Xd + d(1);
    Ydd = (u1/m)*(spsi*sth*cphi - cpsi*sphi) - (P.k2/m)*Yd + d(2);
    Zdd = (u1/m)*cth*cphi - P.g - (P.k3/m)*Zd + d(3);

    % --- Phuong trinh tu the ---
    phidd   = thetad*psid*((P.Iyy - P.Izz)/P.Ixx) + u2/P.Ixx + d(4);
    thetadd = phid*psid*((P.Izz - P.Ixx)/P.Iyy)   + u3/P.Iyy + d(5);
    psidd   = phid*thetad*((P.Ixx - P.Iyy)/P.Izz) + u4/P.Izz + d(6);
    
    xdot = [Xd; Yd; Zd; Xdd; Ydd; Zdd; phid; thetad; psid; phidd; thetadd; psidd];

end
