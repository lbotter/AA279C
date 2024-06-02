function [Mc,alpha] = controlLinear(qEst,qTarget,alphaOld,kp,kd)
%CONTROLFUNCTION Function to compute the momentum to keep the desired
%attitude
%   INPUT:  qEst     :  estimated attitude quat     [4x1]
%           qTarget  :  desired attitude            [4x1]
%           alphaOld :  previous error              [3x1]
%           kp       :  prop. control coefficient
%           kd       :  der. control coefficient
%   OUTPUT: Mc       :  control momentum            [3x1]



[qErr,~]=attitudeError(qTarget,qEst);
alpha=quat2eul(qErr);
alphaDot=(alpha-alphaOld)/dt;
Mc=-kp*alpha'-kd*alphaDot;




end

