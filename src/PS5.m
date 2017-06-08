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
triad1 = [[1 0 0]' [0 0 1]' [0 1 0]']; %y-axis out of orbit plane == UNSTABLE!!!
%triad1 = [1 0 0; 0 1 0; 0 0 1]; %principle axes
triad2 = [1 0 0; 0 1 0; 0 0 1]; %global axes
A0 = getDCM(triad1, triad2); %initial DCM
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
t_total = 0.8*90*60;

wz = n0; %mean motion
w0 = [0 -n0 0]'; %if rotate about y, use -n0. If robot about z, use +n0
M0 = [0 0 0]';
A_b2p = S_top.DCM_B2P;
options = simset('SrcWorkspace','current');
sim('sixDOF_SIM_perturbs',[],options);
[ XYZ ] = getXYZpostprocess( A_DCM, Xout );
[ RTN ] = getRTNpostprocess( A_DCM, error_DCM, Xout );
[yaw, pitch, roll] = dcm2angle( A_DCM ); %expressed in inertial

% PS 5.2
%plotting
figure;hold on
subplot(3,1,1); plot(tout,[Mad_p]); title('Atm Drag'); ylabel('Moment (kN*km)');xlabel('time (s)')
subplot(3,1,2); plot(tout,[Mgg_p]); title('Grav Grad'); ylabel('Moment (kN*km)');xlabel('time (s)')
subplot(3,1,3); plot(tout,[Mm_p]); title('Mag Torq'); ylabel('Moment (kN*km)');xlabel('time (s)')

figure; plot(tout, Moments); title('PS 5.2) Total Moments'); ylabel('Moment (kN*km)');xlabel('time (s)')
figure; plot(tout, XYZ); title('PS 5.2) XYZ position'); ylabel('Distance (km)');xlabel('time (s)')
legend('x','y','z')
figure; plot(tout, RTN); title('PS 5.2) RTN position'); ylabel('Distance (km)');xlabel('time (s)')
legend('x','y','z')
figure; plot(tout, [yaw, pitch, roll]); title('PS 5.2) euler angles'); ylabel('Ang. Vel (rad/s)');xlabel('time (s)')
legend('yaw','pitch','roll')

% PS 5.3 attitude control error - error_DCM
perm = permute(error_DCM, [3 1 2]);
flat_error = reshape(perm, [], 9);
figure; plot(tout, flat_error); title('PS 5.3) Att control error');xlabel('time (s)')
% PS 5.4
[yaw, pitch, roll] = dcm2angle( error_DCM ); %expressed in inertial
figure; plot(tout, [yaw, pitch, roll]); title('Error in euler angles');xlabel('time (s)'); ylabel('Ang. Vel (rad/s)')
legend('yaw','pitch','roll')

% PS 5.5
fprintf('... man idfk\n')