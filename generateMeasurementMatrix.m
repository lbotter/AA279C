function [V,M,w] = generateMeasurementMatrix(x,currentTime,noise)
%UNTITLED Function to Generate measurement matrix, and estimates the
%expected attitude

% Initialize M and V matrixes
M=[];
V=[];
w=[];


% Modeling magnetic field
r_N = (1/1000)*[x(8); x(9); x(10)]; % ECI position km
t = currentTime;
[g, h] = IGRF13;    % Gauss coeffs
alpha_G_0 = 0;      % Greenwich initial longitude
n = 8;              % 5th order (LEO)

[B_N]  = earthmagfield13(r_N, t, g, h, alpha_G_0, n); % Mag field compoents in ECI (Tesla)

[R_P2I] = principal2Inertial([x(1); x(2); x(3); x(4)]);
R_I2P = R_P2I.';
B_P = R_I2P*B_N;    % transform magnetic field to principal coordinates

% Add noise
B_P=addNoise(B_P,noise);

% Append the measurement to V
V=[V;B_N];
M=[M;B_P];
w=[w;1];


% Modeling sun vector
sunPointingVectorECI = [-1; 0; 0];
sunPointingVectorPrincipal = R_I2P*sunPointingVectorECI;
sunPointingVectorPrincipal=addNoise(sunPointingVectorPrincipal,noise);

V=[V,sunPointingVectorECI];
M=[M,sunPointingVectorPrincipal];
w=[w,1];



end