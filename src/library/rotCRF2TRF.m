function [ R ] = rotCRF2TRF( GMST )
% input in radians
% rotz uses degrees
R = rotz(-GMST/pi*180); %rotz uses opposite notation as our class
end

