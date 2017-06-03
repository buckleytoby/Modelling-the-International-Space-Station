function [ S_top ] = genGeom( chd )
%GENGEOM Summary of this function goes here
%   Detailed explanation goes here

% top parent
S_top = struct;


S_top.children = chd;
% calculate DCM's between cuboids and parent center
% -- %
numCubs = length(S_top.children);


% calculate inertia tensor for each cuboid w.r.t. parent center
S_top.tauGlobal = 0; %global inertia tensor
for i=1:numCubs
  S_top.children{i}.tauLocal = cuboidInertia(S_top.children{i}.L, ...
      S_top.children{i}.W, S_top.children{i}.H, S_top.children{i}.mass);
  S_top.children{i}.tauGlobal = convertToGlobalInertia(S_top.children{i}.mass, ...
      S_top.children{i}.tauLocal, -S_top.children{i}.cm_x, -S_top.children{i}.cm_y, ...
      -S_top.children{i}.cm_z, S_top.children{i}.r_x, S_top.children{i}.r_y, S_top.children{i}.r_z);
  S_top.tauGlobal = S_top.tauGlobal + S_top.children{i}.tauGlobal;
end
% calculate center of mass of entire system
S_top = calcSystemMass(S_top);

%body axis surface CM coords
for i=1:numCubs
  S_top.children{i}.surfaceCM_body = S_top.children{i}.surfaceCM_parent + ...
          -1*repmat([S_top.cm_x; S_top.cm_y; S_top.cm_z], [1,6]);
end


% translate final tauGlobal to center of mass
S_top.tauCM = convertToGlobalInertia(S_top.totMass, S_top.tauGlobal, ...
    S_top.cm_x, S_top.cm_y, S_top.cm_z, 0, 0, 0);

% calculate principle axes (eigenvalue/vector problem)
[V,D] = eig(S_top.tauCM);
S_top.tauCM_P = D;
S_top.DCM_P2B = V; %direction cosine matrix - principle to body
S_top.DCM_B2P = S_top.DCM_P2B';

% convert m to km 
S_top.tauGlobal = S_top.tauGlobal / 1000^2;
S_top.tauCM = S_top.tauCM / 1000^2;
S_top.tauCM_P = S_top.tauCM_P / 1000^2;
end

