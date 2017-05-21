function [ A_dot ] = getDCM_dot( w, A )
%GETDCM_DOT calculate time-derivative of DCM
% w is ang vel of body axes w.r.t. inertial
A_dot = -[0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0]*A;
%test
end

