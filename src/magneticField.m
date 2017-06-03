function [ Bp ] = magneticField( longitude, latitude, r, A_DCM, GMST)
%MAGNETICTORQUE Summary of this function goes here
%   return magnetic field (B) in principle axes
%I=current, coil of N turns, can get one m per surface
g = [-30186 -2036 0 0 0;
     -1898 2997 1551 0 0;
     1299 -2144 1296 805 0;
     951 807 462 -393 235];
h = [0 5735 0 0 0;
     0 -2124 -37 0 0;
     0 -361 249 -253 0;
     0 148 -264 37 -307];
%a = equatorial radius
a = 6371.2; %km
%r = geocentric distance
%theta = coelevation -> 90deg - latitude
%phi = azimuthal aka east longitude from greenwich
theta = 90/180*pi - latitude;
phi = longitude;
k = 4;
Br = 0; Bth = 0; Bphi = 0;
for n=1:k
  Br2 = 0;
  Bth2 = 0;
  Bphi2 = 0;
  for m=0:n
    Br2 = Br2 + (g(n,m+1)*cos(m*phi) + h(n,m+1)*sin(m*phi))*getPnm(theta,n,m);
    Bth2 = Bth2 + (g(n,m+1)*cos(m*phi) + h(n,m+1)*sin(m*phi))*getdPnm(theta,n,m);
    Bphi2 = Bphi2 + m*(-g(n,m+1)*sin(m*phi) + h(n,m+1)*cos(m*phi))*getPnm(theta,n,m);
  end
  Br = Br + (a/r)^(n+2)*(n+1)*Br2;
  Bth = Bth - (a/r)^(n+2)*Bth2;
  Bphi = Bphi + (a/r)^(n+2)*Bphi2;
end
Bphi = Bphi*-1/sin(theta);

delta = 90/180*pi - theta; %declination
%alpha = right ascension, alphaG = right ascension of greenwich meridian wertz Appendix J
alphaG = GMST; %is actually GST, but GMST is CLOSE ENOUGH
alpha = phi + alphaG;
B1 = (Br*cos(delta) + Bth*sin(delta))*cos(alpha) - Bphi*sin(alpha);
B2 = (Br*cos(delta) + Bth*sin(delta))*sin(alpha) + Bphi*cos(alpha);
B3 = Br*sin(delta) - Bth*cos(delta);
A = getP2IfromA_DCM(A_DCM);
Bp = A'*[B1 B2 B3]';

end

function dP = getdPnm(theta,n,m)
if n==0 && m==0
  dP=0;
elseif n==m
  dP = sin(theta)*getdPnm(theta,n-1,m-1)+cos(theta)*getPnm(theta,n-1,n-1);
else
  if n==1
    K = 0;
  else
    K = ((n-1)^2-m^2)/((2*n-1)*(2*n-3));
  end
  if K == 0
    dP = cos(theta)*getdPnm(theta,n-1,m)-sin(theta)*getPnm(theta,n-1,m);
  else
    dP = cos(theta)*getdPnm(theta,n-1,m)-sin(theta)*getPnm(theta,n-1,m)- ...
        K*getdPnm(theta,n-2,m);
  end
  
end
end

function P = getPnm(theta,n,m)
if n==0 && m==0
  P=1;
elseif n==m
  P = sin(theta)*getPnm(theta,n-1,m-1);
else
  if n==1
    K = 0;
  else
    K = ((n-1)^2-m^2)/((2*n-1)*(2*n-3));
  end
  if K == 0
    P = cos(theta)*getPnm(theta,n-1,m);
  else
    P = cos(theta)*getPnm(theta,n-1,m) - K*getPnm(theta,n-2,m);
  end
end
end
