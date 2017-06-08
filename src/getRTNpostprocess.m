function [ RTN ] = getRTNpostprocess( A_DCM, error_DCM, Xout )
%UNTITLED2 Summary of this function goes here
%   error_DCM should be from principle to RTN
RTN=[];
for i=1:size(error_DCM,3)
  A_p2i = getP2IfromA_DCM(A_DCM(:,:,i));
  RTN(:,i) = error_DCM(:,:,i) * A_p2i' * Xout(i,1:3)';
end
RTN = RTN';
end

