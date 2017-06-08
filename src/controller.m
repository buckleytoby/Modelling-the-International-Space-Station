function [ u ] = controller( w_des, state, PHI, B )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

A = PHI;
C = eye(size(state)); % w = x
p = 2;
Q = p*(C'*C);
R = eye(size(B,2));

[K] = dlqr(A,B,Q,R); %discrete lqr, x_k+1 = A*x_k+B*u_k
u = w_des - K*state;

end

