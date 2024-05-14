function [qi] = angle2Quaternion(theta)
% angle2Quaternion: rotation of theta degrees about the x axis
ax = 1; ay = 0; az = 0;
qi = [cosd(theta/2); 
      ax*sind(theta/2); 
      ay*sind(theta/2); 
      az*sind(theta/2)] / norm([cosd(theta/2); 
                                ax*sind(theta/2); 
                                ay*sind(theta/2); 
                                az*sind(theta/2)]);
end

