function [Aqq, Aee, B] = linearQuat( q, w , dt, Beuler )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
% get the plant, A as fxn of quaternions
B_ = [0 w(3) -w(2) w(1); -w(3) 0 w(1) w(2); w(2) -w(1) 0 w(3); ...
  -w(1) -w(2) -w(3) 0];
Aqq = eye(4) + dt * 0.5 * B_;

% get the control, B as fxn of moments
wx = 0; %Beuler(1,1);
wy = 0; %Beuler(2,2);
wz = 0; %Beuler(3,3); %wx = dt/Ixx*Mx
A_w = 0.5 * dt * [q4 -q3 q2;q3 q4 -q1;-q2 q1 q4; -q1 -q2 -q3];
Aee = A_w;

B = 0.5 * dt * [q4*wx -q3*wy q2*wz;q3*wx q4*wy -q1*wz;-q2*wx q1*wy q4*wz; ...
  -q1*wx -q2*wy -q3*wz];

end

