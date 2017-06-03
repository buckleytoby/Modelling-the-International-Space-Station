function [ output_args ] = wrapTo2Pi( input_args )
%WRAPTO2PI Summary of this function goes here
%   Detailed explanation goes here
output_args = input_args;
while output_args > 2*pi
    output_args = output_args - 2*pi;
end

end

