function [W] = gyroHardware(R,wt)
%   This function takes as input the geometric parameters of the magnetometer
%   and returns the pointing vector in the Principal frame
%   INPUT   R       : radius of the coil
%           wt      : true omega in principal frame  [3x1] as wx,wy,wz
%  OUTPUT   W       :  sensor output [3x1] vector with angular velocity position in
%                      principal frame


A=pi*R^2;
W=zeros(3,1);
c=3e8; % Speed of light

for i=1:3
    tplus=2*pi*R/(c-wt(i)*R);
    tminus=2*pi*R/(c+wt(i)*R);
    W(i)=c^2*(tplus-tminus)/4/A;
end


end