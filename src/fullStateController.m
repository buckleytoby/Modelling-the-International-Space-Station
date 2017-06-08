function [u, K, timePass1]  = fullStateController(state, w_des, w, PHI, Beuler, A_DCM, timePass, K_old, dt)
%#codegen
% x = [quat; ang vel]

triad1 = getRTNTriadfromState( state );
A_RTN = getDCM(triad1, eye(3));
coder.extrinsic('dcm2quat')
q_des = coder.nullcopy(zeros(4,1));
q = coder.nullcopy(zeros(4,1));
q_des = dcm2quat([[1 0 0]' [0 0 1]' [0 1 0]']); %dcm2quat(A_RTN);
q = dcm2quat(A_DCM);
x = [q; w];
x_des = [q_des; w_des];
coder.extrinsic('dlqr')
coder.extrinsic('lqr')
if timePass > 10*dt
  [Aquat, Aee, Bquat] = linearQuat( q, w, dt, Beuler );
  A = [Aquat, Aee; zeros(3,4), PHI];
  B = eye(7); %[zeros(7,4), [Bquat; Beuler ]]; %quaternion ~ 2nd order Moment, omega ~ 1st order Moment
  C = [4*eye(4), zeros(4,3); zeros(3,4), eye(3)];
  p = 2;
  Q = p*(C'*C);
  R = eye(size(B,2));

  
  K = coder.nullcopy(zeros(7));
  K_lqr = coder.nullcopy(zeros(7));
  K = dlqr(A,B,Q,R); %discrete lqr, x_k+1 = A*x_k+B*u_k
  %K_lqr = lqr(A,B,Q,R);
  timePass1 = 0;
else
  K = K_old;
  timePass1 = timePass;
end
uAll = -K*(x - x_des);
u = uAll(5:7);