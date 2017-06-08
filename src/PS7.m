%% 
% ------------------------ PS7 ------------------------
% let state = [p,dp,r,dr,y,dy] 3 angles & rates

% http://renaissance.ucsd.edu/courses/mae207/wie_chap6.pdf
% page 350, ch 6.7 gives the linearized euler equations

% PS 7.2a - linearEuler.m
% PS 7.2b - control moment gyros

% PS 7.2d - initial state uncertainty
x0 = rand(3,1);
P0 = 0.01*eye(3); %I guessed - error covariance matrix
% PS 7.2e - defined similar to P_0 but smaller
Q0 = P0 ./ 10;
% PS 7.2f - sensitivity matrix
H0 = eye(3); %direct mapping
% PS 7.2g - constant measurement error covariance
R0 = (10 /180*pi)^2 * eye(3,3);

% PS 7.3 - simulation & plotting
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
bNoise = 1;
t_total = 0.01*90*60;

%euler inputs
triad1 = [[1 0 0]' [0 0 1]' [0 1 0]']; %y-axis out of orbit plane == UNSTABLE!!!
%triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %global axes
A0 = getDCM(triad1, triad2); %initial DCM, I2P
I_principle = S_top.tauCM_P; %principle moment of inertia tensor

wz = n0; %mean motion
w0 = [0 -2 0]'; %if rotate about y, use -n0. If robot about z, use +n0
M0 = [0 0 0]';
A_b2p = S_top.DCM_B2P;
options = simset('SrcWorkspace','current');
sim('sixDOF_hw7',[],options);
% analysis
state_error = w_true - x_ekf;
% plotting
figure;hold on; title('Ang. Vel. and Errors')
subplot(2,1,1); plot(tout, [w_true(:,1), x_ekf(:,1)]); legend('wx_true','x_ekf')
subplot(2,1,2); plot(tout, state_error(:,1)); legend('wx error')
figure;hold on
subplot(2,1,1); plot(tout, [w_true(:,2), w_meas(:,2), x_ekf(:,2)]);legend('wy_true','y_meas', 'y_ekf')
subplot(2,1,2); plot(tout, state_error(:,2)); legend('wy error')
figure;hold on
subplot(2,1,1); plot(tout, [w_true(:,3), x_ekf(:,3)]);legend('wz_true','z_ekf')
subplot(2,1,2); plot(tout, state_error(:,3)); legend('wz error')

% residuals (y_meas & z_ekf)
y_meas = w_meas;
resi_pre = abs(y_meas - z_ekf_pre);
resi_post = abs(y_meas - z_ekf_post);
figure;plot(tout, [resi_pre(:,2), resi_post(:,2)]);legend('Pre','Post')
xlabel('time (s)');ylabel('ang vel (rad/s)');title('Residuals Pre & Post estimation')


residuals_error = z_ekf_pre - z_ekf_post;
figure;hold on
subplot(2,1,1); plot(tout, [z_ekf_pre, z_ekf_post]); title('Measured and estimated Ang Vel')
%legend('zx_pre','zy_pre','zz_pre','zx_post','zy_post','zz_post')
ylabel('z_{pre} & z_{post} (rad/s)')
subplot(2,1,2); plot(tout, residuals_error);
legend('wx','wy','wz'); title('Residuals Error')
ylabel('Residuals (rad/s)')
% plot error covariance (P) matrix
perm1 = permute(P_ekf, [3 1 2]);
flat_error1 = reshape(perm1, [], 9);
figure; plot(tout, flat_error1); title('Error Covariance Matrix over time')
xlabel('time (s)')
