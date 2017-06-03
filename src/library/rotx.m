function [ output_args ] = rotx( g )
%ROTZ Summary of this function goes here
%   Detailed explanation goes here

%IN DEGREES
output_args = [1 0 0;
               0 cosd(g) -sind(g);
               0 sind(g) cosd(g)];

end

