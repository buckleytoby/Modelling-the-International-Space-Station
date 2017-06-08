%%
% ------------------------ PS2 ------------------------
close all
A0 = zeros(3,3);
I_principle = S_top.tauCM_P; %principle moment of inertia tensor
M0 = [0 0 0]';
w0 = 10/180*pi*rand([3,1]);
w0_28 = w0;
t_total = 1000;
options = simset('SrcWorkspace','current');
sim('eulerSIM',[],options);

% PS 2.1-2.6
PS2_SCRIPT
% PS 2.7
w0 = [0; 0; 10/180*pi*rand()]; % ang vel parallel to principle axes
sim('eulerSIM',[],options);
%plotting
PS2_SCRIPT
% PS 2.8
t_total = 100;
w0 = w0_28;
I_principle(2,2) = I_principle(1,1); %axi-symmetric
sim('eulerSIM',[],options);
% PS 2.9
y = eulerAnalytic( w0, I_principle, tout );
ang_vel_analytic = [y, repmat(w0(3), [size(y,1), 1])];
% figure; plot(tout, ang_vel); title('Ang. Vel.')
% figure; plot(tout, ang_vel_analytic); legend('Numerical','Analytic')
ang_vel_diff = ang_vel - ang_vel_analytic;
figure; plot(tout, ang_vel_diff); title('Difference in Ang. Vel., Numeric vs Analytic')
legend('wx','wy','wz')
xlabel('time (s)'); ylabel('w_{diff} (rad/s)')
max_diff = max(max(ang_vel_diff));
fprintf('Maximum ang. vel. difference: %.3E deg/s\n', max_diff/pi*180)