%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ForwardModel - runs a Forward Model for one time loop.
% Inputs:
% A - State transition matrix
% B - Controller matrix
% C - Observation matrix
% K - Forward Model Gain
% x - state variable
% y - Noisy observations
% u - controller input
%
% Output:
% x - predicted state variable 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xout] = ForwardModel(A,B,C,K,x,y,u)
x = A*x+B*u + K*(y-C*x);
xout = x;