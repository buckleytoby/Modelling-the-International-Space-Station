%the main wrapper
%% setup
clear;clc; close all
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
h=plotearth
hold(h,'on')
plot3(h,Xout(:,1),Xout(:,2),Xout(:,3))

%%
% ------------------------ PS2 ------------------------
close all
A0 = zeros(3,3);
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
M0 = [0 0 0]';
w0 = 10/180*pi*rand([3,1]);
w0_28 = w0;
t_total = 1000;
options = simset('SrcWorkspace','current');
sim('eulerSIM',[],options);

% PS 2.1-2.6
PS2_SCRIPT
% PS 2.7
w0 = [0; 0; 10/180*pi*rand()]; % ang vel parallel to principle axes
sim('eulerSIM',[],options);
%plotting
PS2_SCRIPT
% PS 2.8
t_total = 100;
w0 = w0_28;
I_principle(2,2) = I_principle(1,1); %axi-symmetric
sim('eulerSIM',[],options);
% PS 2.9
y = eulerAnalytic( w0, I_principle, tout );
ang_vel_analytic = [y, repmat(w0(3), [size(y,1), 1])];
% figure; plot(tout, ang_vel); title('Ang. Vel.')
% figure; plot(tout, ang_vel_analytic); legend('Numerical','Analytic')
ang_vel_diff = ang_vel - ang_vel_analytic;
figure; plot(tout, ang_vel_diff); title('Difference in Ang. Vel., Numeric vs Analytic')
legend('wx','wy','wz')
xlabel('time (s)'); ylabel('w_{diff} (rad/s)')
max_diff = max(max(ang_vel_diff));
fprintf('Maximum ang. vel. difference: %.3E deg/s\n', max_diff/pi*180)

%%
% ------------------------ PS3 ------------------------
close all
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
M0 = [0 0 0]';
w0 = 10/180*pi*rand([3,1]); %generate initial ang vel
x_hat = rand([3,1]); x_hat = x_hat / norm(x_hat);
angle = rand*2*pi; %random y_hat direction

% z_hat = cross(x_hat, y_hat);
% triad1 = [x_hat, y_hat, z_hat];
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
t_total = 1000;
options = simset('SrcWorkspace','current');
sim('eulerSIM',[],options);

%plotting
% PS 3.4
L_xyz = I_principle * ang_vel'; %in body axes
L_123 = zeros(size(L_xyz));
w_123 = L_123;
% figure; hold on; grid on
for i=1:size(tout,1)
  L_123(:,i) = getP2IfromA_DCM(A_DCM(:,:,i)) * L_xyz(:,i); %3xn
  w_123(:,i) = getP2IfromA_DCM(A_DCM(:,:,i)) * ang_vel(i,:)'; %3xn
end
% view(3)
figure; plot3(L_123(1,:), L_123(2,:), L_123(3,:)); grid on; axis equal
title('Ang. Mom. over time (no effective change)')

figure; plot3(w_123(1,:), w_123(2,:), w_123(3,:)); grid on; axis equal
title('Ang. Vel. herpolhode')


L_mag = sqrt(L_123(1,:).^2 + L_123(2,:).^2 + L_123(3,:).^2);
L_hat = L_123 ./ repmat(L_mag, [3,1]);
w_mag = sqrt(w_123(1,:).^2 + w_123(2,:).^2 + w_123(3,:).^2);
w_hat = w_123 ./ repmat(w_mag, [3,1]);

figure; plot3(w_hat(1,:), w_hat(2,:), w_hat(3,:)); grid on; axis equal
hold on
for i=1:size(tout,1)
  plot3([0 L_hat(1,i)], [0 L_hat(2,i)], [0 L_hat(3,i)]) %ang mom
end
view(3)
title('Normalized herpolhode with ang. mom. vector')

% PS 3.4c
DCM_B2P = S_top.DCM_P2B';
f1 = figure; grid on
for i=1:size(tout,1)
  clf(f1)
  plotTriad(f1, A_DCM(:,:,i)); %A_DCM is inertial 2 principle
  DCM_I2P = getInertial2BodyDCM( getP2IfromA_DCM(A_DCM(:,:,i)), DCM_B2P );
  plotTriad(f1, DCM_I2P' );
  plotSC(fig1, S_top);
  view(3); grid on; axis equal; xlim([-1 1]); ylim([-1 1]); zlim([-1 1])
  drawnow
end
view(3)

% PS 3.5a
w0 = [0; 0; 10/180*pi*rand()];
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
t_total = 2*pi / w0(3);
options = simset('SrcWorkspace','current');
sim('eulerSIM',[],options);

%plot velocities and angles
figure; plot(tout, ang_vel); title('3.5a) Ang Vel over time')
legend('wx','wy','wz'); xlabel('time (s)'); ylabel('w (rad/s)')
%angles
f1 = figure; grid on
for i=1:size(tout,1)
  plotTriad(f1, A_DCM(:,:,i)); %A_DCM is inertial to principle
end
view(3); grid on; axis equal; xlim([-1 1]); ylim([-1 1]); zlim([-1 1])
legend('x-dir','y-dir','z-dir')
title('3.5a) Principle axes over time. Pure rotation about body-z')

% PS 3.5b










%%
% ------------------------ PS4 ------------------------

% PS 4.1a Single-spin satellite
%config 1
close all
w0=[0 0 0]';
M0 = w0;
perturb = .01*rand([3,1]);
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
t_total = 200;
for i=1:3
  w0(i) = 0.2;
  w0 = w0 + perturb;
  options = simset('SrcWorkspace','current');
  sim('eulerSIM',[],options);
  figure; plot(tout, ang_vel); 
  title(['4.1a) Non-zero axis: ' num2str(i) '. Ang Vel over time'])
  legend('wx','wy','wz')
  xlabel('time (s)'); ylabel('ang vel (rad/s)')
  %transform to euler angles
  [yaw, pitch, roll] = dcm2angle( A_DCM ); %expressed in inertial
  figure; plot(tout, [yaw, pitch, roll]); 
  title(['4.1a) Non-zero axis: ' num2str(i) '. Euler angles over time'])
  legend('Yaw','Pitch','Roll')
  xlabel('time (s)'); ylabel('Angle (rad)')
  
  w0 = [0 0 0]';  
end
fprintf('Rotation about min & max principle axes is asymptotically stable, \nwhile rotation about the middle axis is unstable \n')

% PS 4.2a momentum wheel (dual-spin)
w0=[0 0 0]';
M0 = w0;
perturb = .01*rand([3,1]);
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
t_total = 200;
%momentum wheel
r_hat = [0 0 0]';
Ir = 1e7;
Mr0 = 0;
wr0 = 0.1; %rad/s
wdotr0 = 0;
%if Ir*wr0 > 0.2*(I_principle(
for i=2
  w0(i) = 0.2;
  r_hat(i) = 1; %spin mom wheel about spin axis for equilibrium state
  w0 = w0 + perturb;
  options = simset('SrcWorkspace','current');
  sim('eulerSIM_momWheel',[],options);
  % PS4.2b verify integration using previous tests (taken from PS2)
  %PS2_SCRIPT
  % PS4.2c equilibrium & stability analysis
  figure; plot(tout, ang_vel); 
  title(['4.2c) Non-zero axis: ' num2str(i) '. Ang Vel over time'])
  legend('wx','wy','wz')
  xlabel('time (s)'); ylabel('ang vel (rad/s)')
  %transform to euler angles
  [yaw, pitch, roll] = dcm2angle( A_DCM ); %expressed in inertial
  figure; plot(tout, [yaw, pitch, roll]); 
  title(['4.2c) Non-zero axis: ' num2str(i) '. Euler angles over time'])
  legend('Yaw','Pitch','Roll')
  xlabel('time (s)'); ylabel('Angle (rad)')
  
  
  w0 = [0 0 0]';  
  r_hat = [0 0 0]';
end
fprintf('Now rotation about all directions \nis stable due to momentum wheel stabilization\n')

mu = 3.986e14 / 1000^3;
% PS 4.3 - gravity gradient - gravGradient.m
% PS 4.4 - magnitude of modelled torque consistent
[ M ] = gravGradient( mu, S_top.tauCM_P, 6776, 0, 0, 1 );
fprintf('Should be zero torques: %.2E\n',M)
[ M ] = gravGradient( mu, S_top.tauCM_P, 6776, 0.25, 0.25, 0.93541 );
fprintf('Should be zero torques: %.2E\n',M)
% PS 4.5 - numerically integrate w/grav gradient in RTN
%orbital elements
mu = 3.986e14 / 1000^3;
a0 = 6776; %km
n0 = sqrt(mu/a0^3);
e0 = 0.0004758;
incl0 = 0; %all angles in degrees
OMEGA0 = 0;
omega0 = 0;
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

%euler inputs
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
t_total = 500;

wz = n0; %mean motion
w0 = [0 0 wz]';
M0 = [0 0 0]';
options = simset('SrcWorkspace','current');
sim('sixDOF_SIM',[],options);
[ XYZ ] = getXYZpostprocess( A_DCM, Xout );
%plotting
figure;plot(tout, Moments)
xlabel('time (s)');ylabel('External Moments (kN*km)')
figure;plot(tout, XYZ)
xlabel('time (s)');ylabel('Location principle axes (km)')
fprintf('As shown, the moments are zero while in line with RTN frame\n')
fprintf('Because of this, the principle frame stays in line with RTN frame\n')

% PS 4.6 - numerically integrate w/grav gradient arbitrary IC's
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

%euler inputs
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
t_total = 2*90*60;

wz = n0; %mean motion
w0 = n0*rand(3,1);
M0 = [0 0 0]';
options = simset('SrcWorkspace','current');
sim('sixDOF_SIM',[],options);
[ XYZ ] = getXYZpostprocess( A_DCM, Xout );
[yaw, pitch, roll] = dcm2angle( A_DCM ); %expressed in inertial
% plot external torques w.r.t. time
figure;plot(tout, Moments); title('PS 4.6) External Moments over time')
xlabel('time (s)');ylabel('External Moments (kN*km)')
%without grav gradient:
bGravGrad = 0;
sim('sixDOF_SIM',[],options);
[yaw1, pitch1, roll1] = dcm2angle( A_DCM ); %expressed in inertial
dy = yaw-yaw1;
dp = pitch-pitch1;
dr = roll-roll1;
% plot attitude (euler angles)
figure;hold on
subplot(2,1,1);plot(tout,[yaw,pitch,roll]); title('With grav gradient')
legend('yaw','pitch','roll')
xlabel('time (s)');ylabel('Euler Angles (rad)')
subplot(2,1,2);plot(tout,[yaw1,pitch1,roll1]); title('Without grav gradient')
legend('yaw','pitch','roll')
xlabel('time (s)');ylabel('Euler Angles (rad)')
% subplot(3,1,3);plot(tout,[dy,dp,dr]); title('Difference')
% legend('yaw','pitch','roll')
% xlabel('time (s)');ylabel('Euler Angles (rad)')

% comment on results
fprintf('The gravity gradient produces moments which act on the spacecraft\n')
fprintf('and affect the euler angles, as shown in the plot comparing angles\n')
fprintf('with and without the gravity gradient applied.\n')

% PS 4.7 - calc Ki, plot regions of stability
I=I_principle;
kn = (I(2,2)-I(1,1))/I(3,3);
kt = (I(3,3)-I(1,1))/I(2,2);
kr = (I(3,3)-I(2,2))/I(1,1);
fprintf('kn: %.2E\n',kn)
fprintf('kt: %.2E\n',kt)
fprintf('kr: %.2E\n',kr)
kt2=linspace(-1,1);
kr2=linspace(1,-1);
pitch = repmat(kt2, [100,1])>repmat(kr2', [1,100]);
roll_yaw = (kr2'*kt2 > 0) & ((1+3*repmat(kt2,[100,1])+kr2'*kt2) > (4*sqrt(kr2'*kt2)));
figure;mesh(kt2,kt2,flipud(double(~pitch)));hold on
LI1 = ~roll_yaw & ~pitch;
LI2 = ~roll_yaw & pitch;
mesh(kt2,kt2,flipud(double(LI1)*2))
mesh(kt2,kt2,flipud(double(LI2)*3))
view(2)
xlabel('k_r');ylabel('k_t')
plot3(kr,kt,10,'r*','linewidth',5)
%legend
h = zeros(4, 1);
h(1) = plot(NaN,NaN,'*g');
h(2) = plot(NaN,NaN,'*b');
h(3) = plot(NaN,NaN,'*y');
h(4) = plot(NaN,NaN,'*','color',[0.5 0 0.5]);
h(5) = plot(NaN,NaN,'*r');
legend(h, 'unstable rpy','unstable ry','unstable p','stable','Intl space station');

% PS 4.8 - comment on expected stability, reproduce stable/unstable motion
fprintf('It is expected that both roll and yaw will be unstable, but pitch will be OK\n');
fprintf('This is confirmed by the rpy plot from section 4.6 where pitch is marginally stable\n');
fprintf('But roll & yaw are completely unstable')
%orbital elements
mu = 3.986e14 / 1000^3;
a0 = 6776; %km
n0 = sqrt(mu/a0^3); %in rad/s
e0 = 0.0004758;
incl0 = 0; %all angles in degrees
OMEGA0 = 0;
omega0 = 0;
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

%euler inputs
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
t_total = 5*90*60;

wz = n0; %mean motion
w0 = [0 0 n0]';
M0 = [0 0 0]';
options = simset('SrcWorkspace','current');
sim('sixDOF_SIM',[],options);
[ XYZ ] = getXYZpostprocess( A_DCM, Xout );
[yaw, pitch, roll] = dcm2angle( A_DCM ); %expressed in inertial
figure;hold on
subplot(2,1,1);plot(tout,[yaw,pitch,roll]); title('PS 4.8) No perturb')
legend('yaw','pitch','roll')
w0 = w0 + 0.0007*[1 1 1]';
sim('sixDOF_SIM',[],options);
[ XYZ ] = getXYZpostprocess( A_DCM, Xout );
[yaw, pitch, roll] = dcm2angle( A_DCM ); %expressed in inertial
subplot(2,1,2);plot(tout,[yaw,pitch,roll]); title('PS 4.8) perturbed')
%
fprintf('As shown, after perturbing the satellite, the roll becomes unstable.\n')
% PS 4.9
fprintf('No, change in mass distr not necessary because the instability is\n')
fprintf('very slow. There is plenty of time for small active control to stabilize.\n')

%%
% ------------------------ PS5 ------------------------
% PS 5.1 program perturbations - done

% PS 5.2 numerical integration
%orbital elements
mu = 3.986e14 / 1000^3;
a0 = 6776; %km
n0 = sqrt(mu/a0^3); %in rad/s
e0 = 0.0004758;
incl0 = 0; %all angles in degrees
OMEGA0 = 0;
omega0 = 0;
nu0 = 0;
UTC_0 = [06,02,2017];
tOffset = 0;
%perturbations
J2 = 0;
Re = 6371;
rho0 = 1.225*1000^3; %kg/km^3
wEarth = [0 0 7.292116e-5]'; %rad/s
bGravGrad = 1;
b_perturbs = 1;

%euler inputs
triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
A0 = getDCM(triad1, triad2); %initial DCM
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
t_total = 1*90*60;

wz = n0; %mean motion
w0 = [0 0 n0]';
M0 = [0 0 0]';
A_b2p = S_top.DCM_B2P;
options = simset('SrcWorkspace','current');
sim('sixDOF_SIM_perturbs',[],options);
[ XYZ ] = getXYZpostprocess( A_DCM, Xout );
[yaw, pitch, roll] = dcm2angle( A_DCM ); %expressed in inertial

% PS 5.2
%plotting
figure;hold on
subplot(3,1,1); plot(tout,[Mad_p]); title('Atm Drag')
subplot(3,1,2); plot(tout,[Mgg_p]); title('Grav Grad')
subplot(3,1,3); plot(tout,[Mm_p]); title('Mag Torq')

figure; plot(tout, Moments);
figure; plot(tout, XYZ);
figure; plot(tout, [yaw, pitch, roll])

% PS 5.3 attitude control error - error_DCM
perm = permute(error_DCM, [3 1 2]);
flat_error = reshape(perm, [], 9);
figure; plot(tout, flat_error)
% PS 5.4
[yaw, pitch, roll] = dcm2angle( error_DCM ); %expressed in inertial
figure; plot(tout, [yaw, pitch, roll]); title('Error in euler angles')

% PS 5.5
fprintf('... man idfk\n')




%% 
% ------------------------ PS6 ------------------------































