function [ A ] = getDCM( triad1, triad2 )
%GETDCM calculate direction cosine matrix between two right-handed triads
%   if triad1 = principle frame & triad2 = global frame, then A = DCM FROM
%   global TO principle

% triad = [ x_vector, y_vector, z_vector ], i_vector = 3x1
% v_triad1 = A * v_triad2
% v_triad2 = A' * v_triad1
x1 = triad1(:,1); y1 = triad1(:,2); z1 = triad1(:,3);
x2 = triad2(:,1); y2 = triad2(:,2); z2 = triad2(:,3);
A = [dot(x1, x2), dot(x1, y2), dot(x1, z2); ...
     dot(y1, x2), dot(y1, y2), dot(y1, z2); ...
     dot(z1, x2), dot(z1, y2), dot(z1, z2)];

end

