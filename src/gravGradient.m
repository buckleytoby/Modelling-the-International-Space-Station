function [ M ] = gravGradient( mu, I, R, cx, cy, cz )
%GRAVGRADIENT Summary of this function goes here
% R = R*(cx*x_hat + cy*y_hat + cz*z_hat) 
% <vector from earth center to satellite, expressed in principle axes>
M = 3*mu/(R^3) * [(I(3,3)-I(2,2))*cy*cz; ...
                   (I(1,1)-I(3,3))*cz*cx; ...
                   (I(2,2)-I(1,1))*cx*cy];
%input units should be kN*km = kg*km^2 / s^2
%output moment is EXPRESSED IN PRINCIPLE FRAME (i'm fairly sure)
end

