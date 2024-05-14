function [] = stabilityRegions(I)
%Stability_regions function to plot the stable regions under gravity toqrue
%   INPUT: I inertia tensor in principal axis
%   OUTPUT: plot of the stability regions

% Calculate stability parameters
Kn = (I(2,2) - I(1,1)) / I(3,3)
Kt = (I(3,3) - I(1,1)) / I(2,2)
Kr = (I(3,3) - I(2,2)) / I(1,1)
% Define the range of x and y values
x = linspace(-1, 1, 1000);
y = linspace(-1, 1, 1000);
  
% Compute the conditions for different regions
[kt, kr] = meshgrid(x, y);
pitch = not(kt > kr);
ry1 = not(kt .* kr > 0);
ry2 = not(1 + 3 * kt + kt .* kr > 4 * sqrt(kr .* kt));

% Merge ry1 and ry2 using logical OR (||) operation
ry = ry1 | ry2;

% Plot the mask matrix with transparency
alpha_value = 0.5;  
% Plot each region with transparency
ax=gca;
hold on
ax.XLim = [-1 1];
ax.YLim = [-1 1];
ax.YDir = 'normal';
hold on
h_pitch=imagesc(x,y,1*pitch,'AlphaData',alpha_value);
hold on;
h_ry=imagesc(x,y,2*ry,'AlphaData',alpha_value);
hold on
colormap([1 1 1;1 1 0;0 0 1]);
set(ax,'CLim',[0 2]);
plot([-10 -10], 'y', 'DisplayName', 'Unstable Pitch');
plot([-10 -10], 'b', 'DisplayName', 'Unstable Roll and Yaw');
hold on
% Plot data points
p=plot(Kt, Kr,"DisplayName",'Satellite');
p.Color = 'red';
p.Marker = 'square';
% Add labels and title
xlabel('Kt');
ylabel('Kr');
title('Stability Regions');
legend('Location','best');

end