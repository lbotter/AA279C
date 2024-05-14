function plot_error_ellipsoid(DCM)
% PLOT_ERROR_ELLIPSOID function to plot the error ellipsoid
%   INPUT: DCM rotation error matrix

    % Generate points on a unit sphere
    [x, y, z] = sphere(100);
    unit_sphere = [x(:), y(:), z(:)];

    % Apply the rotation matrix DCM to the unit sphere
    ellipsoid_points = (DCM * unit_sphere')';

    % Plot the ellipsoid
    plot3(ellipsoid_points(:,1),ellipsoid_points(:,2),ellipsoid_points(:,3),'o');
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Error Ellipsoid');

end