function [Mc] = controlLarge(qEst,qTarget,w,f)
%CONTROLLarge Function to compute the momentum to keep the desired
%attitude
%   INPUT:  qEst     :  estimated attitude quat [4x1]
%           qTarget  :  desired attitude        [4x1]
%           w        :  measured angular v      [3x1]
%           f        :  response frequency Hz
%   OUTPUT: Mc       :  control momentum        [3x1]

% Define Proportional and Derivative constants

geometryDefinition;

kp=f^2/Izz;
kd=2*sqrt(Izz*n^2*(Iyy-Ixx)+kp);

AEst=quat2dcm(qEst');
ATarget=quat2dcm(qTarget');
AErr=AEst*ATarget';

Mc=-kp*[AErr(2,3)-AErr(3,2);AErr(3,1)-AErr(1,3);AErr(1,2)-AErr(2,1)]-kd*w;


end