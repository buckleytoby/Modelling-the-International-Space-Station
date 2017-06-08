
close all
% de-tumble
setupCMG;
bControl = 1;
bGravGrad = 1;
b_perturbs = 1;
bNoise = 0;
t_total = 0.2*90*60;
dt = 0.5;
w_des = [0 0 0]';
%run sim
runHW8Sim
w0 = [0 0 0]';
w0 = rand(3,1);
sim('sixDOF_hw8',[],options);
%plot
figure;plot(tout(1:500/dt), w_true(1:500/dt,:));title('PS 9) De-tumble control: Ang. Vel. in principle')
legend('wx','wy','wz')
[ XYZ ] = getXYZpostprocess( A_DCM, Xout );
figure; plot(tout, XYZ); title('PS 9) XYZ position')
legend('x','y','z')
[ RTN ] = getRTNpostprocess( A_DCM, error_DCM, Xout );
figure; plot(tout, RTN); title('PS 9) RTN position')
legend('x','y','z')

perm = permute(error_DCM, [3 1 2]);
flat_error = reshape(perm, [], 9);
figure; plot(tout, flat_error); title('PS 9) Att control error')
figure;plot(tout, reshape(permute(A_DCM, [3,1,2]),[],9));title('DCM Inertial 2 Principle')
