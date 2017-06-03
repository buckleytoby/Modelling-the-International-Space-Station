function [ output_args ] = MJD2GMST( input_args )
%MJD2GMST Summary of this function goes here
%   Detailed explanation goes here
d = input_args - 51544.5;
output_args = 280.4606/180*pi + 360.9856473/180*pi*d; %output is in radians

end

