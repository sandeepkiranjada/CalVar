%%%%%%
%  Final Project: Math 5546  -  Spring 2019
%
%  Due 11:00 AM,  May 14, 2019 
%
%%%%%  Running this script will load 3 systems: SYS1, SYS2 and SYS3
%
%%%%%   
%
%%%%%%%%%%   ASSIGNMENT FOR THE PROJECY   %%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%         dt x(t) = Ax(t) + Bu(t) + Gw(t);
%
%%%   Given Q > 0 and R> 0  for LQR Problem, find the optimal LQR gain K = R^{-1} B^{T} P
%
%%%   Given the noisy sensed output defined by C and E
%
%%%           y(t) = Cx(t) + Ev(t)
%
%%%   find the optimal LQG gain F = Pi C^{T} N^{-1} for the Kalman state estimator
%
%%%   dt x_ e(t) = A_ex_e(t) + Fy(t) + Bu(t) + Gw(t)
%
%%%   where M = G G^{T}  and N = E E^{T}
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%  For each of the 3 systems:
%
% %   [1] Solve the LQR Problems for the optimal feedback gain matrix K
%
% %   [2] Solve the LQG Problems for the optimal observer gain matrix F
%
% %   [3] Plot the open-loop, LQR closed-loop and LQG closed-loop eigenvalues
%
% %   [4] Given the initial condition IC1, IC2, IC3, set ICj_e = ICj + 5
% %         (a) Solve the open-loop system for 0 < t < 10 seconds
% %         (b) Solve the closed-loop LQR system for 0 < t < 10 seconds
% %         (C) Solve the closed-loop LQG system for 0 < t < 10 seconds
% %         (d) Plot x_{1}(t) for all cases (a), (b) and (c) on one plot
%
% %    [5]  Write a summary (PDF) of your results with tables and figures.
%
%%%%% LOAD SYSTEMS %%%%%%%%
%
load SYS1
load SYS2
load SYS3
%
