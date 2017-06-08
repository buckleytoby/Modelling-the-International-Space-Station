function ax = plotSurfNormals(ax, S_top )
%PLOTSC Plot surface normals
%   Detailed explanation goes here
numCubs = length(S_top.children);

for i=1:numCubs
  for j=1:6
    vec = S_top.children{i}.surfaceCM_parent(:,j)';
    vec2 = ( 5*S_top.children{i}.normals(:,j))';
    m = [vec;vec2];
    %plot3(m(:,1),m(:,2),m(:,3),0,'linewidth',4)
    quiver3(vec(1),vec(2),vec(3),vec2(1),vec2(2),vec2(3),2,'linewidth',2)
    plot3(vec(1),vec(2),vec(3),'*','markers',5)
end
axis equal
view(3)
xlabel('x'); ylabel('y'); zlabel('z');
end