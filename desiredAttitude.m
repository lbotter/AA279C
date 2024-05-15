function [q] = desiredAttitude(pointingPrincipal,posECI,q0)
%DESIREDATTITUDE Function to compute the desired attitude in quaternions
%               INPUT: pointingPrincipal: vector to point to in principal axis
%                      posECI: radius of the orbit in Inertial frame
%                      q0 : actual attitude
%               OUTPUT: q : desired attitude of the satellite [4x1]

% Define the pointing in inertial
pointingInertial=principal2Inertial(q0)*pointingPrincipal;
pointingInertialNorm = pointingInertial / norm(pointingInertial);
posECINorm = -posECI / norm(posECI);
% Compute the rotation axis (cross product)
k = cross(pointingInertialNorm, posECINorm);
% Compute the rotation angle (dot product)
cosTheta = dot(pointingInertialNorm,posECINorm);
Theta = acos(cosTheta);

% Normalize the rotation axis
kNorm = k / norm(k);

% Compute the quaternion
qw = cos(Theta / 2);
qxyz = kNorm * sin(Theta / 2);
q = [qw, qxyz];
if q(1) < 0
    q=-q;
end
q=q/norm(q);

if isrow(q)
    q=q';
end

end

