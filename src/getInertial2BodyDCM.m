function [ DCM ] = getInertial2BodyDCM( DCM_princ2inert, DCM_xyz2princ )
%GETINERTIAL2PRINCIPLEDCM Summary of this function goes here
%   Detailed explanation goes here
DCM = DCM_xyz2princ' * DCM_princ2inert';

end

