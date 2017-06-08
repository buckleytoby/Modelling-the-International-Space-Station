
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
A0 = getDCM(triad1, triad2); %initial DCM, I2P
I_principle = S_top.tauCM_P; %principle moment of inertia tensor

wz = n0; %mean motion
w0 = [0 -n0 0]'; %if rotate about y, use -n0. If robot about z, use +n0
M0 = [0 0 0]';
A_b2p = S_top.DCM_B2P;
options = simset('SrcWorkspace','current');
sim('sixDOF_hw6',[],options);