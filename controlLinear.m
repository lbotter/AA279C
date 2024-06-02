function [Mc] = controlLinear(qEst,qTarget,w,f)
%CONTROLFUNCTION Function to compute the momentum to keep the desired
%attitude
%   INPUT:  qEst     :  estimated attitude quat [4x1]
%           qTarget  :  desired attitude        [4x1]
%           w        :  measured angular v      [3x1]
%   OUTPUT: Mc       :  control momentum         [3x1]


geometryDefinition;

kp=f^2/Izz;
kd=2*sqrt(Izz*n^2*(Iyy-Ixx)+kp);

[qErr,~]=attitudeError(qTarget,qEst);
alpha=quat2eul(qErr);

Mc=-kp*alpha'-kd*w;




end

