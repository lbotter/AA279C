function [meas] = addNoise(truth,sensor)
%SENSOR_MEASUREMENT Function to simulate the measurement from a sensor
%   INPUT: truth = ground truth [3x1]
%          stDev = standard deviation for gaussian noise
%   OUTPUT: meas = simulated measure [3x1]

 if isrow(truth)
     truth=truth';
 end
 meas = truth + sensor.bias + normrnd(0,sensor.stDev,3, 1);
 meas=meas/norm(meas);

end

