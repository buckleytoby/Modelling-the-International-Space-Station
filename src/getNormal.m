function [ n ] = getNormal( x, y, z)
%GETNORMAL Summary of this function goes here
% assumes at least 3 sides (no lines)
v1 = [x(2) - x(1), y(2) - y(1), z(2) - z(1)];
v2 = [x(3) - x(1), y(3) - y(1), z(3) - z(1)];
N = cross(v1,v2);
n = N/norm(N);
end

