function [R,T,N] = inertial2RTN(px, py, pz, vInertial)
%LVL_frame function to convert from inertial to RTN frame
%   INPUT: px,py,px position in different directions (body frame)
%          vInertial: velocity in inertial frame
%          A : Matrix from inertial to principal
%   OUTPUT: R,T,N components of the RTN frame in principal frame

    R = ([px; py; pz] / norm([px; py; pz])); %x
    T = (vInertial) / norm(vInertial); %y
    N = cross(R, T); %z
end



