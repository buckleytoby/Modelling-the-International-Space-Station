function [ h ] = plotEarthLongLat()
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
reearth = 6378;
plotcolor='blue';
load('topo.mat','topo');
topoplot=[topo(:,181:360) topo(:,1:180)];
h = contour(-180:179,-90:89,topoplot,[0 0],'black');hold on;
axis equal; grid on; 
set(gca, 'XLim',[-180 180],'Ylim',[-90 90]);
xlabel('Long (deg)'); ylabel('Lat (deg)');
end

