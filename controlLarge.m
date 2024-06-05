function [Mc, alpha] = controlLarge(mean, qTarget, kp, kd)
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

[q0, q1, q2, q3, wx, wy, wz] = deal(mean(1), mean(2), mean(3), mean(4), mean(5), mean(6), mean(7));

qEst = [q0; q1; q2; q3];
alphaDot = [wx; wy; wz];

AEst=quat2dcm(qEst');
ATarget=quat2dcm(qTarget');
AErr=AEst*ATarget';
alpha=[AErr(2,3)-AErr(3,2);AErr(3,1)-AErr(1,3);AErr(1,2)-AErr(2,1)];
Mc=-kp.*[AErr(2,3)-AErr(3,2);AErr(3,1)-AErr(1,3);AErr(1,2)-AErr(2,1)]/2-kd.*alphaDot;


end