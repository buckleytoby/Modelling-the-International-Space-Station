function RTN_triad = getRTNTriadfromState(state)

% R direction in xECI
xECI = state(1:3);
R = xECI / norm(xECI);
% not quite T in vel direction
vECI = state(4:6);
not_T = vECI / norm(vECI);
% N is the cross product
N = cross(R, not_T);
% get the actual T
T = cross(N, R);
RTN_triad = [R, N, T]; %should be 3x3 now
end