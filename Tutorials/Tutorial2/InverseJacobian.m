clc
clear all

syms S E W Ls Le Lw

J = [-Ls*sin(S)-Le*sin(S+E)-Lw*sin(S+E+W), -Le*sin(S+E)-Lw*sin(S+E+W), -Lw*sin(S+E+W);...
     Ls*cos(S)+Le*cos(S+E)+Lw*cos(S+E+W), Le*cos(S+E)+Lw*cos(S+E+W), Lw*cos(S+E+W);...
     1,1,1];
 
 Jinv = inv(J);