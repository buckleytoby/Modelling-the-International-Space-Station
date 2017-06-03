function [r_eci, v_eci] = OE2XV(a, e, i, OMEGA, omega, nu)
%compute position & velocity in ECI given the orbital elements
%assume we want mu in km^3/s^2
%output in km and km/s
% NOTE! ALL IN DEGREES
mu = 3.986e14 / 1000^3;
n = sqrt(mu/a^3);
E = 2*atan2d(sqrt(1-e)*tand(nu/2), sqrt(1+e));
r_pqw = [a*(cosd(E)-e), a*sqrt(1-e^2)*sind(E) 0]';
v_pqw = a*n/(1-e*cosd(E)).*[-sind(E), sqrt(1-e^2)*cosd(E) 0]';

%matlab builtin fxns rotz, rotx uses opposite notation from
%class ... so use opposite angle (no extra negative)
R = rotz(OMEGA)*rotx(i)*rotz(omega);
r_eci = R*r_pqw;
v_eci = R*v_pqw;
end
