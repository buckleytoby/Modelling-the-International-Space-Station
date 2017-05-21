function fig1 = plotSC(fig1, S_top )
%PLOTSC Plot spacecraft
%   Detailed explanation goes here
numCubs = length(S_top.children);
figure(fig1);
for i=1:numCubs
  x = S_top.children{i}.x + S_top.children{i}.t_x;
  y = S_top.children{i}.y + S_top.children{i}.t_y;
  z = S_top.children{i}.z + S_top.children{i}.t_z;
  patch(x,y,z,'cyan')
end
axis equal
view(3)
xlabel('x'); ylabel('y'); zlabel('z');
end
