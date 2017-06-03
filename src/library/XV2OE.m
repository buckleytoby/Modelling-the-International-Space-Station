function [a, e, incl, OMEGA, omega, nu] = XV2OE(xECI, vECI)
%compute position & velocity in ECI given the orbital elements
%assume we want mu in km^3/s^2
%output in km and km/s
mu = 3.986e14 / 1000^3;
a = 0; e = 0; incl = 0; OMEGA = 0; omega = 0; nu = 0; %set so that simulink is happy

h = cross(xECI, vECI);
h_abs = norm(h);
W = [h(1) -h(2) h(3)] ./ h_abs;
incl = atan2(sqrt(W(1)^2+W(2)^2), W(3));
if ~((W(3) > 0 && -pi/2 < incl < pi/2) || (W(3) < 0 && pi/2 < incl < 3*pi/2))
    disp('Problem with XV2OE: incl')
    incl
    return
end
OMEGA = atan2(W(1), W(2));
if ~((W(2) > 0 && -pi/2 < OMEGA < pi/2) || (W(2) < 0 && pi/2 < OMEGA < 3*pi/2))
    disp('Problem with XV2OE: OMEGA')
    OMEGA
    return
end
p = h_abs^2 / mu;

eps = norm(vECI)^2/2 - mu./norm(xECI);
a = -mu/2/eps;
n = sqrt(mu/a^3);
e = sqrt(1-p/a);

den = 1-norm(xECI)/a;
E = atan2(dot(xECI,vECI)/(a^2*n), den);
if ~((den > 0 && -pi/2 < E < pi/2) || (den < 0 && pi/2 < E < 3*pi/2))
    disp('Problem with XV2OE: E')
    E
    return
end
M = E - e*sin(E);
den = sqrt(1-e);
nu = 2*atan2(sqrt(1+e)*tan(E/2), den);
if ~((den > 0 && -pi/2 < nu < pi/2) || (den < 0 && pi/2 < nu < 3*pi/2))
    disp('Problem with XV2OE: nu')
    nu
    return
end
den = xECI(1)*cos(OMEGA)+xECI(2)*sin(OMEGA);
u = atan2(xECI(3)/sin(incl), den);
if ~((den > 0 && -pi/2 < u < pi/2) || (den < 0 && pi/2 < u < 3*pi/2))
    disp('Problem with XV2OE: u')
    u
    return
end
omega = u-nu;
% R = rotz(OMEGA)*rotx(incl)*rotz(omega)';
% r_pqw = p./(1+e.*cos(nu)); %I think I still need to rotate this...
% r = r_pqw;
% 
% x = r.*cos(nu);
% y = r.*sin(nu);

%convert to degrees
OMEGA = OMEGA /pi*180;
omega = omega /pi*180;
nu = nu /pi*180;
end
