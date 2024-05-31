function [meas] = addNoise(truth, sensor)
%SENSOR_MEASUREMENT Function to simulate the measurement from a sensor
%   INPUT: truth = ground truth [1x1]
%          stDev = standard deviation for gaussian noise
%   OUTPUT: meas = simulated measure [1x1]

truth = truth(:);

meas = truth + sensor.bias + normrnd(0, sensor.stDev, 1, 1);

end