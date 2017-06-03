function [ XYZ ] = getXYZpostprocess( A_DCM, Xout )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
XYZ=[];
for i=1:size(A_DCM,3)
  A_p2i = getP2IfromA_DCM(A_DCM(:,:,i));
  XYZ(:,i) = A_p2i'*Xout(i,1:3)';
end
XYZ = XYZ';
end

