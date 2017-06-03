function [ Fad, Mad ] = atmDragTorque( A_b2p, surfaceAreasList, surfaceCM_bodyList, ...
  normalsList, rE_sc, A_DCM, wEarth, V_sc )
%atmDragTorque Summary of this function goes here
%   rE_sc = position of spacecraft w.r.t. earth (expressed in inertial)
%   A_DCM = DCM output from kinematic equations
%   wEarth = rotation rate of earth
%   V_sc = velocity of spacecraft w.r.t. inertial (ECI)
%
%   Returns:
%      Fad = inertial force due to atm drag
%      Mad = inertial moment due to atm drag

Cd = 1.28; %flat plate
rho = getRho( norm(rE_sc) );
A_p2i = getP2IfromA_DCM(A_DCM);
F_total = [0 0 0]';
M_total = [0 0 0]';
F = [0 0 0]';

for i=1:length(surfaceAreasList)
  rj = surfaceCM_bodyList(:,i);
  rj_inertial = A_p2i * A_b2p * rj; %body -> principle -> inertial
  A = surfaceAreasList(i);
  n = normalsList(:,i); %in body
  n_inertial = A_p2i * A_b2p * n; %body -> principle -> inertial
  V_air = cross(wEarth, rE_sc);
  VdA_air = V_sc - V_air;
  if dot(rj_inertial, VdA_air) < 0 %not facing velocity direction
    continue
  end
  Vabs = norm(VdA_air);
  F = -1/2 * Cd * rho * Vabs^2 * (dot(VdA_air, n_inertial)/Vabs) * VdA_air/Vabs * A;
  F_total = F_total + F;
  M_total = M_total + cross(rj_inertial, F);
end
Fad = F_total;
Mad = M_total;
end

