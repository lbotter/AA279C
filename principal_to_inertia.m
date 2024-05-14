function [R_B2I] = principal_to_inertia(q)
%PRINCIPAL_TO_INERTIA Generates the matrix to go from principal to inertial
%frame
%   INPUT : quaternions
%   OUTPUT : rotation matrix
q0c=q(1);
q1c=q(2);
q2c=q(3);
q3c=q(4);

R_B2I = [q0c^2 + q1c^2 - q2c^2 - q3c^2   2*(q1c*q2c + q0c*q3c)           2*(q1c*q3c - q0c*q2c);
         2*(q1c*q2c - q0c*q3c)           q0c^2 - q1c^2 + q2c^2 - q3c^2   2*(q2c*q3c + q0c*q1c);
         2*(q1c*q3c + q0c*q2c)           2*(q2c*q3c - q0c*q1c)           q0c^2 - q1c^2 - q2c^2 + q3c^2].';
end

