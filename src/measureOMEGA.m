function w_meas = measureOMEGA(w_true, DCM_B2P, bNoise)
% measure the angular velocity
% pretend there's one gyro on each body axis of ISS
% wertz page 267 eq 7-139 has method for calculating ang vel with many
% gyros

% w_true is from the euler eq's, in principle axes
w_body = DCM_B2P' * w_true;

if bNoise
  %wertz pg 201: .01 deg/s
  stdDev = 10 /180*pi;
  w_body = w_body + normrnd(0, stdDev*eye(size(w_body)));
end


w_meas = DCM_B2P * w_body;