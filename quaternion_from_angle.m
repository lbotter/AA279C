function [qi] = quaternion_from_angle(theta)
%QUATERNION_FROM_ANGLE Summary of this function goes here
ax = 1; ay = 0; az = 0;
qi = [cosd(theta/2); ax*sind(theta/2); ay*sind(theta/2); az*sind(theta/2)] / norm([cosd(theta/2); ax*sind(theta/2); ay*sind(theta/2); az*sind(theta/2)]);
end

