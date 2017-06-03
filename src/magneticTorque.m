function [ Mm_p ] = magneticTorque( A_b2p, surfaceAreasList, normalsList, longitude, ...
  latitude, r, A_DCM, GMST)
%MAGNETICTORQUE Summary of this function goes here
%   return moment about center of mass expressed in principle frame
%S_sat = %surface contained by coil
%m_max = 4*pi*10e-7*1*S_sat*0.1;
%from wertz eq 6-18 can use m=N*I*A*n instead, n=unit normal, A=area,
Bp = magneticField( longitude, latitude, r, A_DCM, GMST); %doesn't change much for each surface
M_total = [0 0 0]';

for i=1:length(surfaceAreasList)
  A = surfaceAreasList(i);
  n = A_b2p * normalsList(:,i); %transform normals from body axis to principle axis
  m = 1 * 0.1 * A * n;
  M = cross(m, Bp); %both in principle axes
  M_total = M_total + M;
end

Mm_p = M_total;