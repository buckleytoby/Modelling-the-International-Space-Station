function [ error_DCM ] = getAttError( A_DCM, state)
%UNTITLED2 Summary of this function goes here
%   attitude error in the form of a DCM

triad2 = A_DCM';
triad1 = getRTNTriadfromState( state );

error_DCM = getDCM(triad1, triad2); %this DCM is from principle 2 RTN

end

