function [V,M,w] = generateMeasurementMatrix(x,currentTime,R_P2I)
%UNTITLED Function to Generate measurement matrix, and estimates the
%expected attitude

% Initialize M and V matrixes
M=[];
V=[];
w=[];

% Compute rotation matrices
R_I2P = R_P2I.';

% Initialize sensor's biases and stDev
sensorsDefinition;

% SENSOR 1: MAGNETOMETER
% Compute real magnetic field
B_N=magneticFieldECI(x,currentTime);
B_P = R_I2P*B_N;    % transform magnetic field to principal coordinates
% Add noise
B_P=addNoise(B_P,magSensor);

% Append the measurement to V
V=[V;B_N];
M=[M;B_P];
w=[w;magSensor.w];


% SENSOR 2: SUN SENSOR
% Modeling sun vector
sunPointingVectorECI = [-1; 0; 0];
sunPointingVectorPrincipal = R_I2P*sunPointingVectorECI;
%Add nouise
sunPointingVectorPrincipal=addNoise(sunPointingVectorPrincipal,sunSensor);


V=[V,sunPointingVectorECI];
M=[M,sunPointingVectorPrincipal];
w=[w,1];



end