% Script to solve problm 5
close all;
clc;
clear;
% Load the parameters
Params
%%
% Test the error function

% pointing vector of the antenna in principal axis
r=[0.0630,-0.9979,0.0136];
q_desired=[0 1 0 0];
% 

q_desired=q_desired/norm(q_desired);

% Load the parameters
runSim;
%
plot_Results_PS5;