function [meas] = addNoise(truth,sensor)
%SENSOR_MEASUREMENT Function to simulate the measurement from a sensor
%   INPUT: truth = ground truth [3x1]
%          stDev = standard deviation for gaussian noise
%   OUTPUT: meas = simulated measure [3x1]

 if isrow(truth)
     truth=truth';
 end
 if isscalar(truth)==1
     addit=sensor.bias1;
 else
     addit=sensor.bias;
 end
 meas = truth + addit + normrnd(0,sensor.stDev,length(truth), 1);
 %meas=meas/norm(meas);

end

