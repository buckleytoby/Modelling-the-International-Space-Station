function [ chd ] = mkChild( L,W,H,m,t_x,t_y,t_z,r_x,r_y,r_z )
%MKCHILD Summary of this function goes here
%   Detailed explanation goes here
% generate cuboids of correct size
chd = struct;
chd.L = L;
chd.W = W;
chd.H = H;
[x,y,z,area,CM] = getCuboid(L, W, H); %meters
chd.x = x;
chd.y = y;
chd.z = z;
chd.surfaceAreas = area;
chd.surfaceCM_local = CM;
chd.mass = m; %kg





% define position of cuboids w.r.t. parent center
count = 1;
chd.t_x = t_x; %translate x - from cuboid to parent
chd.t_y = t_y; %translate y
chd.t_z = t_z; %translate z
chd.cm_x = chd.t_x + chd.L/2;
chd.cm_y = chd.t_y + chd.W/2;
chd.cm_z = chd.t_z + chd.H/2;
chd.r_x = r_x; %rotate x
chd.r_y = r_y; %rotate y
chd.r_z = r_z; %rotate z
%generate surface normals & positions
chd.normals = getNormals(x,y,z);
chd.surfaceCM_parent = chd.surfaceCM_local + ...
      repmat([chd.t_x; chd.t_y; chd.t_z], [1 6]); %this is in parent center coords

end

