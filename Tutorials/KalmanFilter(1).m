%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KalmanFilter - runs a Kalman Filter for one time loop.
% Inputs:
% A - State transition matrix
% B - Controller matrix
% C - Observation matrix
% Sigma_nu - System noise covariance matrix
% Sigma_omega - Observation noise covariance matrix
% x - state variable
% P - error covariance matrix
% y - Noisy observations
% u - controller input
%
% Output:
% x - predicted state variable over time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xout P] = KalmanFilter(A,B,C,Sigma_nu,Sigma_omega,x,P,y,u)

    % Prediction
    x_Prior = A*x+B*u;
    P = A*P*A'+B*B'*Sigma_nu;

    % Correction
    L = P*C'/(C*P*C'+Sigma_omega);
    x = x_Prior + L*(y-C*x_Prior);
    P = (eye(2)-L*C)*P;
    
    xout = x;