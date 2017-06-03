function E = calcE(M, e, tol)
%calculate eccentric anomaly given mean anomaly, eccentricity
%and a tolerance
%all angles in radians
E = zeros(size(M));
for i=1:length(M)
E(i) = M(i);
d = 1+tol;
while abs(d) > tol
  d = - (E(i) - e*sin(E(i)) - M(i)) / (1 - e*cos(E(i)));
  E(i) = E(i) + d;
end
end