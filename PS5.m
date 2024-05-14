% Script to solve problm 5
close all;
clc;
clear;
% Load the parameters
Params;
% Edit the inertia tensor
    % % Original
    % Ixx = 1.0e+03 * 0.230242103134935;
    % Iyy = 1.0e+03 * 5.293968876196306;
    % Izz = 1.0e+03 * 5.518363350668759;
    % % Edited
    % %Edit 1
    % Ixx = 1.0e+03 * 1.230242103134935;
    % Iyy = 1.0e+03 * 1.693968876196306;
    % Izz = 1.0e+03 * 1.518363350668759;
    % Edit 2
    Ixx = 1.0e+03 * 1.930242103134935;
    Iyy = 1.0e+03 * 1.193968876196306;
    Izz = 1.0e+03 * 1.618363350668759;

I_principal=[Ixx 0 0;0 Iyy 0;0 0 Izz];
% Plot the stable regions
stability_regions(I_principal);
%%
% Run over time
Integrator;
%%
plot_Results_PS5;