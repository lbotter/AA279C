function [q_e,DCM] = attitudeError(q_d,q_a)
%ATTITUDE_ERROR Function that computes the rotation matrix from desired
%   attitude to actual attitude
% INPUT: q_d desired attitude (4x1 quaternion)
%        q_a actual attitude (4x1 quaternion)
% OUTPUT: q_e error between the two attitudes
%           DCM attitude error DCM

if size(q_d,2)~=4
    q_d=q_d';
end

if size(q_a,2)~=4
    q_a=q_a';
end

q_d=q_d/norm(q_d);
q_a=q_a/norm(q_a);

q_e=quatmultiply(quatconj(q_d), q_a);
q_e=q_e/norm(q_e);
DCM=quat2dcm(q_e);

end

