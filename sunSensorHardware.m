function [V] = sunSensorHardware(alpha,beta,RP2I,sunVec)
%SUNSENSORHARDWARE modeling of the hardware behavior of a sun sensor
%   This function takes as input the geometric parameters of the sun vector
%   and returns the pointing vector in the Principal frame
%   INPUT   alpha   :  angle of mounting of the photocells
%           beta    :  angle of view of each cell
%           RP2I    :  matrix from principal to inertial frame
%           sunVec  :  direction of the sun in the inertial frame
%  OUTPUT   V       :  sensor output [3x1] vector with sun position in
%                      principal frame

% Generate directions in principal frame for sensor pointing
absCoeff=0.8;
S=0.001;
c=absCoeff*S;

sensorDirection=eye(3);
sunVec=(RP2I'*sunVec)/norm(RP2I'*sunVec);
Imax=c*cos((+beta-2*alpha)/beta*pi/2);
Imin=-cos((2*alpha-beta)/beta*pi/2)*c;
slope=(Imax-Imin)/(2*beta-2*alpha);
v=zeros(3,1);
for i=1:3
    theta=atan2(norm(cross(sensorDirection(:,i),sunVec)), dot(sensorDirection(:,i),sunVec));
    I1=c*cos((theta-alpha)/beta*pi/2);
    I2=-cos((theta+alpha)/beta*pi/2)*c;
    % Measured current differential
    Itot=I1+I2;
    % Angle estimate
    angle=(alpha-beta)+(Itot-Imin)/slope;
    v(i)=cos(angle);
end
v=v/norm(v);


end

