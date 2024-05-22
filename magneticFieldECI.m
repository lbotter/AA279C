function [B_N] = magneticFieldECI(x,currentTime)
%MAGNETICFIELDPRINCIPAL Function to compute the magnetic field in principal
%axis
%   INPUT: x the state
%   OUTPUT: B_P magnetic field in principal frame

% Modeling magnetic field
r_N = (1/1000)*[x(8); x(9); x(10)]; % ECI position km
t = currentTime;
[g, h] = IGRF13;    % Gauss coeffs
alpha_G_0 = 0;      % Greenwich initial longitude
n = 8;              % 5th order (LEO)
[B_N]  = earthmagfield13(r_N, t, g, h, alpha_G_0, n); % Mag field compoents in ECI (Tesla)


end

