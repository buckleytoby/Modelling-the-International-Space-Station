function [ x_k, P_k, z_k_, z_k ] = EKF( x_k_1, y_k, u_k_1, PHI_k_1, B, ...
            P_k_1, Q_k_1, R_k, H_k)
%EKF Summary of this function goes here
% P[mxm] = uncertainty of state parameters, Pii = (sigma_ii)**2
% Q[mxm] = uncertainty of dynamis model
% R[nxn] = uncertainty of measurements
% H[nxm] = sensitivity matrix
% x[mx1] = state vector
% y[nx1] = measurement vector
% z[nx1] = estimated (what the internal model predicts) = H*x

% predict step
x_k_ = PHI_k_1 * x_k_1 + B * u_k_1;
P_k_ = PHI_k_1 * P_k_1 * PHI_k_1' + Q_k_1;

% update step / measurement update
z_k_ = H_k * x_k_;
K_k = P_k_*H_k' / (H_k*P_k_*H_k' + R_k);
x_k = x_k_ + K_k*(y_k - z_k_);
z_k = H_k * x_k;
t = K_k*H_k;
P_k = (eye(size(t)) - t)*P_k_*(eye(size(t)) - t)' + K_k*R_k*K_k';

end

