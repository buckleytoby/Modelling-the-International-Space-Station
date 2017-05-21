function S_top = calcSystemMass(S_top)
%CALCSYSTEMMASS calculate system mass and CM
mass = 0;
mx = 0; my = 0; mz = 0;
numCubs = length(S_top.children);
for i=1:numCubs
  mass = mass + S_top.children{i}.mass;
  mx = mx + S_top.children{i}.cm_x * S_top.children{i}.mass;
  my = my + S_top.children{i}.cm_y * S_top.children{i}.mass;
  mz = mz + S_top.children{i}.cm_z * S_top.children{i}.mass;
end
S_top.totMass = mass;
S_top.cm_x = mx / mass; %from parent to CM
S_top.cm_y = my / mass;
S_top.cm_z = mz / mass;

end