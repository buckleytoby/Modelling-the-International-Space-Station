function [ rho ] = getRho( r )
%getRho Summary of this function goes here
%   r - position from center of earth to satellite
%   h0 - equatorial radius (6378.137 km)
%   H - characteristic height (10 km)
%   rho0 - sea level density (1.225*1000^3 kg/km^3)
%   From aa279a PS 8.3
h0 = 6378.137;
H = 10;
rho0 = 1.225*1000^3;
if (r-h0) > 0
  rho = rho0*exp(-(r - h0)/H);
else
  rho = rho0;
end

end

