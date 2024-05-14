function [M] = gravityGradient(I_principal, R, Rnorm)
% Function to compute the gravity gradient momentum
% INPUT: I_principal axis
%        R: [cx,cy,cx] Position in RTN normalized
%        Rnorm: radius length (orbit radius)
% OUTPUT: Mx,My,Mz in principal axis to feed to Euler Equations
mu = 3.98600e14;

Ixx = I_principal(1,1);
Iyy = I_principal(2,2);
Izz = I_principal(3,3);

M = 3*mu/Rnorm^3*[(Izz-Iyy)*R(2)*R(3);
                  (Ixx-Izz)*R(3)*R(1);
                  (Iyy-Ixx)*R(1)*R(2)];

end

