%% 
% ------------------------ PS6 ------------------------

% GPS
% 4 thrusters - 13.3 kg-f (29.3 lbf) https://www.quora.com/How-does-the-International-Space-Station-maintain-its-orbit-and-what-propellant-does-it-use
% https://en.wikipedia.org/wiki/Attitude_control
% https://www.youtube.com/watch?v=9XDRTysaCww
%   -> 4 control moment gyroscopes, 200 lbs
% https://space.stackexchange.com/questions/566/do-any-spacecraft-use-gnss-for-attitude-determination
%   -> 4 GPS receivers

% PS 6.2a determinstic method

% PS 6.2b q-method

% PS 6.2c ang vel measurements & reconstruction of A_DCM through kinematic
% eq's
%       --> assume gyros (IMU's) and horizon sensor


% PS 6.3 plotting estimated attitude
% run sim
bNoise = 0;
t_total = 0.1*90*60;
run6DOFSim_ps6

error1 = zeros(3,3,size(A_DCM,3));
error2 = error1;
error3 = error1;
error4 = error1;
for i=1:size(A_DCM,3)
  error1(:,:,i) = getDCM(A_i2p_meas(:,:,i)' , A_DCM(:,:,i)' );
  error2(:,:,i) = getDCM(A_i2p_meas2(:,:,i)' , A_DCM(:,:,i)' );
  error3(:,:,i) = getDCM(A_i2p_meas3(:,:,i)' , A_DCM(:,:,i)' );
  error4(:,:,i) = getDCM(A_DCM_meas(:,:,i)' , A_DCM(:,:,i)' );
end
perm1 = permute(error1, [3 1 2]);
perm2 = permute(error2, [3 1 2]);
perm3 = permute(error3, [3 1 2]);
perm4 = permute(error4, [3 1 2]);
flat_error1 = reshape(perm1, [], 9);
flat_error2 = reshape(perm2, [], 9);
flat_error3 = reshape(perm3, [], 9);
flat_error4 = reshape(perm4, [], 9);
figure;plot(tout, reshape(permute(A_i2p_meas, [3 1 2]),[],9));title('PS 6.3) Estimated attitude with no errors')
figure; hold on
subplot(4,1,1); plot(tout, flat_error1); title('PS 6.3) Att meas error - Algebra')
subplot(4,1,2); plot(tout, flat_error2); title('PS 6.3) Att meas error - Algebra distributed')
subplot(4,1,3); plot(tout, flat_error3); title('PS 6.3) Att meas error - Statistical')
subplot(4,1,4); plot(tout, flat_error4); title('PS 6.3) Att meas error - integrated from measured ang. vel.')

% PS 6.4 introduce sensor errors - constant bias & gaussian noise
bNoise = 1;
t_total = 0.1*90*60;
run6DOFSim_ps6

error1 = zeros(3,3,size(A_DCM,3));
error2 = error1;
error3 = error1;
error4 = error1;
for i=1:size(A_DCM,3)
  error1(:,:,i) = getDCM(A_i2p_meas(:,:,i)' , A_DCM(:,:,i)' );
  error2(:,:,i) = getDCM(A_i2p_meas2(:,:,i)' , A_DCM(:,:,i)' );
  error3(:,:,i) = getDCM(A_i2p_meas3(:,:,i)' , A_DCM(:,:,i)' );
  error4(:,:,i) = getDCM(A_DCM_meas(:,:,i)' , A_DCM(:,:,i)' );
end
perm1 = permute(error1, [3 1 2]);
perm2 = permute(error2, [3 1 2]);
perm3 = permute(error3, [3 1 2]);
perm4 = permute(error4, [3 1 2]);
flat_error1 = reshape(perm1, [], 9);
flat_error2 = reshape(perm2, [], 9);
flat_error3 = reshape(perm3, [], 9);
flat_error4 = reshape(perm4, [], 9);
figure;plot(tout, reshape(permute(A_i2p_meas, [3 1 2]),[],9));title('PS 6.5) Estimated attitude with sensor errors')
figure; hold on
subplot(4,1,1); plot(tout, flat_error1); title('PS 6.5) Att meas error - Algebra')
subplot(4,1,2); plot(tout, flat_error2); title('PS 6.5) Att meas error - Algebra distributed')
subplot(4,1,3); plot(tout, flat_error3); title('PS 6.5) Att meas error - Statistical')
subplot(4,1,4); plot(tout, flat_error4); title('PS 6.5) Att meas error - integrated from measured ang. vel.')

% PS 6.6

% PS 6.7 - attitude control error using measured attitude
perm = permute(error_DCM, [3 1 2]);
flat_error = reshape(perm, [], 9);
figure; plot(tout, flat_error); title('PS 6.7) Att control error')
[yaw, pitch, roll] = dcm2angle( error_DCM ); %expressed in inertial
figure; plot(tout, [yaw, pitch, roll]); title('PS 6.7) Error in euler angles')
legend('yaw','pitch','roll')