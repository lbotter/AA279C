function [q] = desiredAttitude(v1Principal, v2Principal, v1Inertial, v2Inertial)
% DESIREDATTITUDEMULTIVECTORS Function to compute the desired attitude in quaternions
%                             This function aligns v1 and v2
%               INPUT: v1Principal  :     first vector in the principal frame
%                      v2Principal  :     second vector in the principal frame
%                      v1Inertial   :     first vector in the inertial frame
%                      v2Inertial   :     second vector in the inertial frame

%               OUTPUT: q           :     desired attitude of the satellite [4x1]

% Normalize the input vectors
v1Principal = v1Principal' / norm(v1Principal);
v2Principal = v2Principal' / norm(v2Principal);
v1Inertial = v1Inertial' / norm(v1Inertial);
v2Inertial = v2Inertial' / norm(v2Inertial);

% Align the first pair of vectors
q1 = alignVectors(v1Principal, v1Inertial);

% Rotate the second principal vector by q1
v2PrincipalRotated = quatRotate(q1, v2Principal);

% Align the rotated second principal vector with the second inertial vector
q2 = alignVectors(v2PrincipalRotated, v2Inertial);

% Combine the two rotations
q = quatMultiply(q2, q1);

% Normalize the output quaternion
q = q / norm(q);

% Ensure the output is a column vector
if isrow(q)
    q = q';
end

end

function q = alignVectors(v1, v2)
% ALIGNVECTORS Function to compute the quaternion that aligns vector v1 with vector v2
    k = cross(v1, v2);
    cosTheta = dot(v1, v2);
    Theta = acos(cosTheta);
    kNorm = k / norm(k);
    qw = cos(Theta / 2);
    qxyz = kNorm * sin(Theta / 2);
    q = [qw, qxyz];
    if q(1) < 0
        q = -q;
    end
    q = q / norm(q);
end

function vRot = quatRotate(q, v)
% QUATROTATE Function to rotate vector v by quaternion q
    qConj = [q(1), -q(2), -q(3), -q(4)];
    vQuat = [0, v];
    vRotQuat = quatMultiply(quatMultiply(q, vQuat), qConj);
    vRot = vRotQuat(2:4);
end

function q = quatMultiply(q1, q2)
% QUATMULTIPLY Function to multiply two quaternions
    qw = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
    qx = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
    qy = q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
    qz = q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1);
    q = [qw, qx, qy, qz];
end

