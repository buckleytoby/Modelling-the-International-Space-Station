
m1 = 1000; %kg
m2 = m1; m3 = m1;
r1 = 5/1000; %km
r2 = r1; r3 = r1;
h1 = r1 / 20;
wCMG = 691; %rad/s - from pdf
Lw = 0.5*[m1*r1^2*wCMG; m2*r2^2*wCMG; m3*r3^2*wCMG];
Ix = m1/12*(3*r1^2+h1^2);
Iz = m1*r1^2/2;
Iw = [ Iz 0 0;0 Ix 0; 0 0 Ix] + [Ix 0 0; 0 Iz 0; 0 0 Ix] + [Ix 0 0;0 Ix 0;0 0 Iz];
% update inertia matrix of ISS to include CMG's
S_top.tauCM = S_top.tauCM + Iw;
[V,D] = eig(S_top.tauCM);
S_top.tauCM_P = D;
S_top.DCM_P2B = V; %direction cosine matrix - principle to body
S_top.DCM_B2P = S_top.DCM_P2B';

A_CMG0 = S_top.DCM_B2P*eye(3);