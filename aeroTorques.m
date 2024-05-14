function M = aeroTorques(x, geometryPrincipalFrame)
% OUTPUT: Mx,My,Mz in principal axis
globeRadius = 6371000;

R_B2P = [0.00384827072457221        0.0138855978754232          0.99989618510327;
        -0.998008358547225       -0.0629053976529481       0.00471457484817906;
         0.0629643318260823        -0.997922893372914        0.0136158657428925];

cgBody = [2.4901184e-01;  1.1191946e-01;  2.7926924e+00];
cgPrincipal = R_B2P*cgBody;

xECI = x(8);
yECI = x(9);
zECI = x(10);
vx = x(11);
vy = x(12);
vz = x(13);

omegaEarthECI = [0;0;7.292115*10^-5];
svPosECI = [xECI; yECI; zECI]; % satellite vehicle ECI position (m)

[R_P2I] = principal2Inertial([x(1); x(2); x(3); x(4)]);
R_I2P = R_P2I.';
velECI = R_P2I*[vx; vy; vz];

velAirECI = velECI - cross(omegaEarthECI, svPosECI);
velAirPrincipal = R_I2P*velAirECI;
velAirPrincipalUnit = velAirPrincipal ./ norm(velAirPrincipal);

momentTotal = [0;0;0];

cD = 1.28; % drag coeff. of flat plate
rho = 3.88E-13; % MSISE-90 air density, mean solar activity (kg/m^3)

for i = 1:size(geometryPrincipalFrame, 1)
    currRow = geometryPrincipalFrame(i,:);

    coordinates = currRow(1:3);
    area = currRow(4);
    unitNormal = currRow(5:7);

    coordinates = coordinates(:);
    unitNormal = unitNormal(:);

    force = (-1/2)*cD*rho*(norm(velAirECI)^2)*dot(velAirPrincipalUnit,unitNormal)*velAirPrincipalUnit * area;
    
    % check if surface is pointing in same direction as velocity vector
    if dot(velAirPrincipalUnit, unitNormal) <= 0
        force = [0;0;0];
    end

    moment = cross(coordinates-cgPrincipal, force);
    momentTotal = momentTotal + moment;
end

M = momentTotal;

end