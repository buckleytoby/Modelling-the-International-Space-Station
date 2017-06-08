function betterPlotEarth()
%UNTITLED17 Summary of this function goes here
%   Detailed explanation goes here
grs80 = referenceEllipsoid('grs80','km');
domeRadius =  3000;  % km
domeLat =  39;       % degrees
domeLon = -77;       % degrees
domeAlt = 0;         % km

[x,y,z] = sphere(20);
xEast  = domeRadius * x;
yNorth = domeRadius * y;
zUp    = domeRadius * z;
zUp(zUp < 0) = 0;

figure('Renderer','opengl')
ax = axesm('globe','Geoid',grs80,'Grid','off', ...
    'GLineWidth',1,'GLineStyle','-',...
    'Gcolor',[0.9 0.9 0.1],'Galtitude',100);
ax.Position = [0 0 1 1];
axis equal off
view(3)
load topo
geoshow(topo,topolegend,'DisplayType','texturemap')
demcmap(topo)
land = shaperead('landareas','UseGeoCoords',true);
plotm([land.Lat],[land.Lon],'Color','black')
rivers = shaperead('worldrivers','UseGeoCoords',true);
plotm([rivers.Lat],[rivers.Lon],'Color','blue')

end

