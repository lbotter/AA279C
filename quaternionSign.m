function [qE] = quaternionSign(qReal,qEst)
%QUATERNIONSIGN Function to use the same sign of quaternions for GT and estimate
%       INPUT: qReal : ground truth quaternion
%              qEst  : estimated quaternion
%       OUTPUT:qE  : corrected sign of estimated quaternion

if(qReal(1)*qEst(1)<0)
    qEest=-qEst;
end
qE=qEst;

end

