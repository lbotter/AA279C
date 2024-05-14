function [meas] = addNoise(variable,noise)
%SENSOR_MEASUREMENT Function to simulate the measurement from a sensor
%   INPUT: variable = ground truth
%          noise = measurement noise
%   OUTPUT: meas = simulated measure

 amplitude = noise.* variable;
 meas = variable + amplitude .* rand(size(variable));

end

