figure; grid on; hold on;
globeRadius = 6371000;
[X,Y,Z] = sphere;
X2 = X * globeRadius;
Y2 = Y * globeRadius;
Z2 = Z * globeRadius;
globe = surf(X2,Y2,Z2);
set(globe, 'FaceColor', "#0072BD")
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')

[X,Y,Z] = cylinder([globeRadius, globeRadius], 500);
Z = 2*globeRadius*Z;
M = makehgtform('translate',[0,0,0],'xrotate', pi/4, 'yrotate', pi/2);
h = surf(X,Y,Z,'Parent',hgtransform('Matrix',M),'LineStyle','none','FaceAlpha',0.4);

title("Solar Radiation Pressure Shadowing Visualization")
