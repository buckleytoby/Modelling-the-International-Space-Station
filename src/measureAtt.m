function [ A_i2p_meas, A_i2p_meas2, A_i2p_meas3, V, M ] = measureAtt( bNoise, DCM_B2P, A_DCM, A_DCM_true, state )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% set up antenna array in body frame w.r.t. CM
antennas_CM = [-1.5, -0.75 1;
            -1.5, 0.75 1;
            1.5, -0.75 1;
            1.5, 0.75 1]' / 1000; %could add the CM offset if want to be more realistic
antennas_CM_P = DCM_B2P * antennas_CM;
A_p2i = getP2IfromA_DCM(A_DCM);
A_p2i_true = getP2IfromA_DCM(A_DCM_true);

%assume pre-processing is done in order to get GPS in ECI
V = getGPS(bNoise, A_p2i_true, antennas_CM_P, state);
% get global location of antennas expressed in principle axes
M = antennas_CM_P; % + A_p2i' * repmat(state(1:3), [1, size(antennas_CM_P,2)]);
%now can calculate the measured A_DCM
A_i2p_meas = M / V;

% variant that distributes the error
m1 = M(:,1);
m2 = M(:,2);
m1_ = (m1+m2)/2;
m2_ = (m1-m2)/2;
qm = cross(m1_, m2_);
rm = cross(m1_, qm);
M2 = [m1_, qm, rm];
%set up V
v1 = V(:,1);
v2 = V(:,2);
v1_ = (v1+v2)/2;
v2_ = (v1-v2)/2;
qm = cross(v1_, v2_);
rm = cross(v1_, qm);
V2 = [v1_, qm, rm];
A_i2p_meas2 = M2 / V2;

% q-method
W = ones(size(antennas_CM));
U = ones(size(antennas_CM));
w = ones(size(antennas_CM, 2), 1); %weights vector, for now do uniform
for i=1:size(antennas_CM, 2)
  W(:,i) = sqrt(w(i))*M(:,i);
  U(:,i) = sqrt(w(i))*V(:,i);
end
B = W*U';
S = B+B';
Z = [B(2,3)-B(3,2), B(3,1)-B(1,3), B(1,2)-B(2,1)]';
sigma = trace(B);
K = [S-eye(size(S))*sigma, Z; Z', sigma];

[Veig, D] = eig(K);
Veig = real(Veig);
D = real(D);
[Y,~] = max(D);
[~,ind] = max(Y);
q = Veig(:, ind(1)); %ind is 2 long because D is 4x4 diagonal

coder.extrinsic('quat2dcm')
A_i2p_meas3 = coder.nullcopy(zeros(3,3));
A_i2p_meas3 = quat2dcm(q');

end

