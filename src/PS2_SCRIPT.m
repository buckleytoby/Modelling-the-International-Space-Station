% ----------------------------------
% post processing and plotting for all the problem set 2 questions


% post-process
L = I_principle * ang_vel';
T_rot = 0.5 * sum(ang_vel .* L', 2) ;
% momentum ellipsoid of w0
xr = norm(L(:,1)) / I_principle(1,1);
yr = norm(L(:,1)) / I_principle(2,2);
zr = norm(L(:,1)) / I_principle(3,3);
[xm, ym, zm] = ellipsoid(0,0,0,xr,yr,zr);
% energy ellipsoid of w0
xr = sqrt(2*T_rot(1) / I_principle(1,1));
yr = sqrt(2*T_rot(1) / I_principle(2,2));
zr = sqrt(2*T_rot(1) / I_principle(3,3));
[xe, ye, ze] = ellipsoid(0,0,0,xr,yr,zr);

% plotting
figure; plot(tout, ang_vel); title('Anguler Velocity of Simulation');ylabel('ang. vel. (rad/s');xlabel('time (s)')
legend('wx','wy','wz')
figure; plot(tout, T_rot); title('Rotational Energy');ylabel('Energy (MJ');xlabel('time (s)')
figure; surf(xm, ym, zm); axis equal; title('Mom. Ellipsoid and Polhode')
hold on; surf(xe, ye, ze);
% PS 2.4 - 3d plot of polhode
plot3(ang_vel(:,1), ang_vel(:,2), ang_vel(:,3),'linewidth',5)
plot3(-ang_vel(:,1), -ang_vel(:,2), -ang_vel(:,3),'linewidth',5)
% PS 2.5 - 2d plots on principle axes
figure; plot(ang_vel(:,1), ang_vel(:,2)); hold on; plot(-ang_vel(:,1), -ang_vel(:,2));
title('2d polhode - XY plane'); xlabel('wx'); ylabel('wy'); axis equal
figure; plot(ang_vel(:,1), ang_vel(:,3)); hold on; plot(-ang_vel(:,1), -ang_vel(:,3));
title('2d polhode - XZ plane'); xlabel('wx'); ylabel('wz'); axis equal
figure; plot(ang_vel(:,2), ang_vel(:,3)); hold on; plot(-ang_vel(:,2), -ang_vel(:,3));
title('2d polhode - YZ plane'); xlabel('wy'); ylabel('wz'); axis equal
% PS 2.6 - phase plots (wx vs wx_dot, etc.)
figure; plot(ang_vel(:,1), ang_accel(:,1)); title('X - phase plane')
xlabel('w_x'); ylabel('w_/dot_x')
figure; plot(ang_vel(:,2), ang_accel(:,2)); title('Y - phase plane')
xlabel('w_y'); ylabel('w_/dot_y')
figure; plot(ang_vel(:,3), ang_accel(:,3)); title('Z - phase plane')
xlabel('w_z'); ylabel('w_/dot_z')