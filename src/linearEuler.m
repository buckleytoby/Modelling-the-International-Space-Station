function [ PHI, B ] = linearEuler( w, dt, I_principle )
%LINEAREULER Summary of this function goes here
%   Detailed explanation goes here
Ix = I_principle(1,1);
Iy = I_principle(2,2);
Iz = I_principle(3,3);
kx = (Iz-Iy)/Ix;
ky = (Ix-Iz)/Iy;
kz = (Iy-Ix)/Iz;
A_ = [0 -kx*w(3) -kx*w(2);
      -ky*w(3) 0 -ky*w(1);
      -kz*w(2) -kz*w(1) 0];
B_ = [1/Ix 0 0;0 1/Iy 0; 0 0 1/Iz];

PHI = (dt * A_ + eye(3));
B = (dt * B_);

end

