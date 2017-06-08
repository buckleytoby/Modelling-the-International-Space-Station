function [ q_dot ] = getDCM_dot( w, q )
%GETDCM_DOT calculate time-derivative of DCM
% w is ang vel of principle axes w.r.t. inertial EXPRESSED IN PRINCIPLE FRAME
B = [0 w(3) -w(2) w(1); -w(3) 0 w(1) w(2); w(2) -w(1) 0 w(3); ...
  -w(1) -w(2) -w(3) 0];
q_dot = 0.5 * B * q;
%test
end

