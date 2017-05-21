function [ y ] = eulerAnalytic( w0, I, tspan )
%EULERANALYTIC analytic solution of euler equations

%axially symmetric - assumes Ix == Iy
if (I(1,1) - I(2,2)) < 1e-3
  lambda = (I(3,3) - I(1,1)) * w0(3) / I(1,1);
  odefun = @(t,y) [-lambda*y(2), lambda*y(1)]'; %x-y euler eq soln
  [~, y] = ode113( odefun ,tspan, w0(1:2));

end
end

