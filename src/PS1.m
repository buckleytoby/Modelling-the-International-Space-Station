%% setup
clear;clc; close all
% cd('./aa279c/aa279c_github/src')
addpath('./library')
% ISS length: 72.8m width: 108.5m height: 20m

chd = {};
%main truss
chd{end+1} = mkChild(3,108.5,3,1000,-1.5,-108.5/2,-1.5,0,0,0);
%pressurized area
chd{end+1} = mkChild(3,3,45,1000,1.5,-1.5,-15,0,0,0);
%solar panel
chd{end+1} = mkChild(3, 17, 0.1, 1000, 1.5, 2, 25, 0,0,0);
chd{end+1} = mkChild(3, 17, 0.1, 1000, 1.5, -19, 25, 0,0,0);

chd{end+1} = mkChild(3, 14, 0.1, 1000, 1.5, 2, 15, 0,0,0);
chd{end+1} = mkChild(3, 14, 0.1, 1000, 1.5, -16, 15, 0,0,0);

%top solar panels
chd{end+1} = mkChild(35,12,0.5,1000,2,-50,0,0,0,0);
chd{end+1} = mkChild(35,12,0.5,1000,2,-37,0,0,0,0);
chd{end+1} = mkChild(35,12,0.5,1000,2, 38,0,0,0,0);
chd{end+1} = mkChild(35,12,0.5,1000,2, 25,0,0,0,0);

%bottom solar panels
chd{end+1} = mkChild(35,12,0.5,1000,-37,-50,0,0,0,0);
chd{end+1} = mkChild(35,12,0.5,1000,-37,-37,0,0,0,0);
chd{end+1} = mkChild(35,12,0.5,1000,-37, 38,0,0,0,0);
chd{end+1} = mkChild(35,12,0.5,1000,-37, 25,0,0,0,0);


%generate geometry
S_top = genGeom(chd);
[ surfaceAreasList, surfaceCM_bodyList, normalsList ] = S_top2Lists( S_top );


fig1 = figure;hold on;
fig1 = plotSC(fig1, S_top);
fig1 = plotAxesTriads(fig1, S_top, 10);
grid on

%plot surface normals
fig2 = figure;hold on;
fig2 = plotSC(fig2, S_top);
fig2 = plotSurfNormals(fig2, S_top);grid on;title('ISS with surface normals')

% PS1.9: Orbit Propagation
%orbital elements
mu = 3.986e14 / 1000^3;
a0 = 6776; %km
n0 = sqrt(mu/a0^3); %in rad/s
e0 = 0.0004758;
incl0 = 51.6397; %all angles in degrees
OMEGA0 = 106.0758;
omega0 = 233.6745;
nu0 = 0;
%perturbations
J2 = 0;
Re = 6371;
rho0 = 1.225*1000^3; %kg/km^3
H = 10;
M = 1500; %kg
Cd = 0;
A = 20/1000^2; %km
B = Cd*A/M;
h0 = Re;
wEarth0 = [0 0 7.292116e-5]'; %rad/s
bGravGrad = 1;
numDaysSim = 0.1;
options = simset('SrcWorkspace','current');
sim('orbitSIM',[],options);

% plot earth and orbit
betterPlotEarth()

hold on;plot3(Xout(:,1),Xout(:,2),Xout(:,3),'linewidth',8);
view(2)
ylim([-7000 7000]);title('Earth and ISS orbit');legend('ISS orbit')




