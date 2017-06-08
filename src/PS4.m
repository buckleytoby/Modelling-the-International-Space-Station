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
Ir = 1e7 / 1000^2;
Mr0 = 0;
wr0 = 0.1; %rad/s
wdotr0 = 0;
%if Ir*wr0 > 0.2*(I_principle(
for i=1:3
  w0(i) = 0.2;
  r_hat(i) = 1; %spin mom wheel about spin axis for equilibrium state
  w0 = w0 + perturb;
  options = simset('SrcWorkspace','current');
  sim('eulerSIM_momWheel',[],options);
  % PS4.2c equilibrium & stability analysis
  figure; plot(tout, ang_vel); 
  title(['4.2c) Non-zero axis: ' num2str(i) '. Ang Vel over time with Momentum Wheel'])
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
% PS4.2b verify integration using previous tests (taken from PS2)
PS2_SCRIPT
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
figure;plot(tout, Moments);title('PS 4.5) Moments when aligned with RTN frame')
xlabel('time (s)');ylabel('External Moments (kN*km)')
xlabel('time (s)')
figure;plot(tout, XYZ);title('Location in principle axes - in line with RTN')
xlabel('time (s)');ylabel('Location (km)')
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