% Script to solve problm 5
close all;
clc;
clear;
% Load the parameters

%%
% Test the error function
q_desired=[1 0 0 0]';
q_desired=q_desired/norm(q_desired);
[qe,DCM]=attitude_error(q_desired,q0);
plot_error_ellipsoid(DCM);

% Load the parameters
runSim;
%
plot_Results_PS5;