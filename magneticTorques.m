function M = magneticTorques(x, currentTime)
% OUTPUT: Mx,My,Mz in principal axis

r_N = (1/1000)*[x(8); x(9); x(10)]; % ECI position km
t = currentTime;
[g, h] = IGRF13;    % Gauss coeffs
alpha_G_0 = 0;      % Greenwich initial longitude
n = 8;              % 5th order (LEO)

[B_N]  = earthmagfield13(r_N, t, g, h, alpha_G_0, n); % Mag field compoents in ECI (Tesla)

[R_P2I] = principal2Inertial([x(1); x(2); x(3); x(4)]);
R_I2P = R_P2I.';
B_P = R_I2P*B_N;    % transform magnetic field to principal coordinates

mMax = 4*pi*10^(-7) * 10000 * 38 * 10; % estimate of satellite residual magnetism
m = mMax*[0;0;1]; % satellite magnetic moment (Nm/T)


M = cross(m, B_P);

end