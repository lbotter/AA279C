% Script to define sensors' parameters and bias
% Sensors are strucured array with parameters:
%          bias     : a vector with the sensor bias
%          stDev    : the standard deviation of gaussian noise
%          w        : weight to add to the measurement

% SUNSENSOR
sunSensor.bias = 0;
sunSensor.stDev = deg2rad(0.3);
sunSensor.w = 1;

% MAGNETOMETER
magSensor.bias = 12e-12;
magSensor.stDev = 5e-9;
magSensor.w = 1;

% GYROSCOPE
gyroSensor.bias = deg2rad(0.05);
gyroSensor.stDev = deg2rad(0.03);
gyroSensor.w = 1;

