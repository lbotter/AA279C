% Script to define sensors' parameters and bias
% Sensors are strucured array with parameters:
%          bias     : a vector with the sensor bias
%          stDev    : the standard deviation of gaussian noise
%          w        : weight to add to the measurement

% SUNSENSOR
sunSensor.bias=[0.0003;0.0002;0];
sunSensor.bias1=0;
sunSensor.stDev=deg2rad(0.3);
sunSensor.w=1;

% MAGNETOMETER
magSensor.bias=[0;0;0];
magSensor.bias1=12e-12;
magSensor.stDev=5e-9;
magSensor.w=1;

% GYROSCOPE
gyroSensor.bias=0*[1,1,1]*1e-6;
gyroSensor.stDev=deg2rad(0.3);
gyroSensor.bias1=deg2rad(0.5);
gyroSensor.w=1;

