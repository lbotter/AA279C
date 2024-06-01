function [M] = gravityGradient(I_principal,x)
% Function to compute the gravity gradient momentum
% INPUT: I_principal axis
%        x : state vector
%        Rnorm: radius length (orbit radius)
% OUTPUT: Mx,My,Mz in principal axis to feed to Euler Equations

R_P2I = principal2Inertial(x(1:4));
mu = 3.98600e14;
% Velocity in inertial frame from principal
vInertial = R_P2I*[x(11); x(12); x(13)];
[R,~,~] = inertial2RTN(x(8),x(9),x(10),vInertial);
% ASK albert
R_I2P = inv(R_P2I);
R = R_I2P*R;
Rnorm=norm([x(8); x(9); x(10)]);

Ixx = I_principal(1,1);
Iyy = I_principal(2,2);
Izz = I_principal(3,3);

M = 3*mu/Rnorm^3*[(Izz-Iyy)*R(2)*R(3);
                  (Ixx-Izz)*R(3)*R(1);
                  (Iyy-Ixx)*R(1)*R(2)];

end

