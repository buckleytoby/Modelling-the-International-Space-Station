function tauGlobal = convertToGlobalInertia(m, tauLocal, tx, ty, tz, rx, ry, rz)
% 1-2-3 rotation
r1 = rotx(rx); %in degrees
r2 = roty(ry);
r3 = rotz(rz);
R = r3*r2*r1;
tau = R*tauLocal*(R');
tauGlobal = tau + m*[ty^2+tz^2 -tx*ty -tx*tz;
                     -tx*ty tx^2+tz^2 -ty*tz;
                     -tx*tz -ty*tz tx^2+ty^2];
end