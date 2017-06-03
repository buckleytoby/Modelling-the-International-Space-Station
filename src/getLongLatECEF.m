function [ psi, lambda, recef ] = getLongLatECEF( xECI, GMST )
%GETLONGLATECEF Summary of this function goes here
%   get longitude (psi), latitude (lambda), and r_ecef from xECI and GMST
recef = zeros(size(xECI));

R = rotCRF2TRF(GMST);
rxyz = R*xECI;
psi = asin(rxyz(3)/norm(rxyz));
lambda = atan2(rxyz(2), rxyz(1));
r = norm(rxyz);
rMag = r;
recef(:) = [r*cos(psi)*cos(lambda) ...
              r*cos(psi)*sin(lambda) ...
              r*sin(psi)]';

end

