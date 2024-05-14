function M = solarTorques(x, geometryPrincipalFrame)
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

[R_P2I] = principal2Inertial([x(1); x(2); x(3); x(4)]);
R_I2P = R_P2I.';
velECI = R_P2I*[vx; vy; vz];
svPosECI = [xECI; yECI; zECI]; % satellite vehicle ECI position (m)
sunPointingVectorECI = [-1; 0; 0];

sunPointingVectorPrincipal = R_I2P*sunPointingVectorECI;
earthPointingVectorPrincipal = R_I2P * (-1*svPosECI / norm(svPosECI));

% absorption, specular reflection, diffuse reflection coeffs.
cA = (1/3);
cS = (1/3);
cD = (1/3);

% pSun = 1358; % (N/m^2 or W/m^2) mean solar momentum flux
% pAlbedo = 600;

pSun = 4.56E-6; % 4.5 microPascal
pAlbedo = (600/1358)*pSun;

momentTotal = [0;0;0];

for i = 1:size(geometryPrincipalFrame, 1)
    currRow = geometryPrincipalFrame(i,:);

    coordinates = currRow(1:3);
    area = currRow(4);
    unitNormal = currRow(5:7);

    coordinates = coordinates(:);
    unitNormal = unitNormal(:);

    thetaSun = acos(dot(sunPointingVectorPrincipal,unitNormal) / (norm(sunPointingVectorPrincipal)*norm(unitNormal)));
    thetaEarth = acos(dot(earthPointingVectorPrincipal,unitNormal) / (norm(earthPointingVectorPrincipal)*norm(unitNormal)));
    
    forceSun = -pSun * ((1-cS)*sunPointingVectorPrincipal + 2*(cS*cos(thetaSun) + (1/3)*cD)*unitNormal) * cos(thetaSun) * area;
    forceAlbedo = -pAlbedo * ((1-cS)*earthPointingVectorPrincipal + 2*(cS*cos(thetaEarth) + (1/3)*cD)*unitNormal) * cos(thetaEarth) * area;
    
    % check if surface is pointing towards sun or earth
    if dot(sunPointingVectorPrincipal, unitNormal) <= 0
        forceSun = [0;0;0];
    end
    if dot(earthPointingVectorPrincipal, unitNormal) <= 0
        forceAlbedo = [0;0;0];
    end

    momentSun = cross(coordinates-cgPrincipal, forceSun);
    momentAlbedo = cross(coordinates-cgPrincipal, forceAlbedo);

    momentTotal = momentTotal + momentSun + momentAlbedo;
end

M = momentTotal;

% check for earth shadow
if sqrt(yECI^2 + zECI^2) < globeRadius && xECI > 0
    M = [0; 0; 0];
end

end