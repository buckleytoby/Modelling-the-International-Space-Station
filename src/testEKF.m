%test EKF...
x = x0;
P = P0;
P_ekf5 = zeros(3,3,length(tout));
for i=1:length(tout)
  y_meas = w_meas(i,:)';
  [ PHI, B ] = linearEuler( w_meas(i,:)', .05, I_principle );
  [ x_k, P_ekf1, z_ekf_pre, z_ekf_post ] = EKF( x, y_meas, [0 0 0]', PHI, B, ...
            P, Q0, R0, H0);
  x = x_k;
  P = P_ekf1;
  P_ekf5(:,:,i) = P_ekf1;
end


perm1 = permute(P_ekf5, [3 1 2]);
flat_error1 = reshape(perm1, [], 9);
figure; plot(tout, flat_error1); title('Error Covariance Matrix over time')