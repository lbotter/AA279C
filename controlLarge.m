function [Mc,alpha] = controlLarge(qEst,qTarget,alphaOld,dt,kp,kd)
%CONTROLLarge Function to compute the momentum to keep the desired
%attitude
%   INPUT:  qEst     :  estimated attitude quat [4x1]
%           qTarget  :  desired attitude        [4x1]
%           w        :  measured angular v      [3x1]
%           alphaOld :  previous error          [3x1]
%           dt       :  time step s
%           kp       :  prop. control coefficient
%           kd       :  der. control coefficient
%   OUTPUT: Mc       :  control momentum        [3x1]
%           alpha    :  angle error


AEst=quat2dcm(qEst');
ATarget=quat2dcm(qTarget');
AErr=AEst*ATarget';
alpha=[AErr(2,3)-AErr(3,2);AErr(3,1)-AErr(1,3);AErr(1,2)-AErr(2,1)];
alphaDot=(alpha-alphaOld)/dt;
Mc=-kp.*[AErr(2,3)-AErr(3,2);AErr(3,1)-AErr(1,3);AErr(1,2)-AErr(2,1)]/2-kd.*alphaDot;


end