function [ output_args ] = rotz( g )
%ROTZ Summary of this function goes here
%   Detailed explanation goes here
%IN DEGREES
output_args = [cosd(g) -sind(g) 0;
               sind(g) cosd(g) 0;
               0 0 1];

end

