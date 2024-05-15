% Parameters definition

% Satellite Mass
m = 260;
% Cavendish constant for Earth
mu = 3.986004418e14;
%Initial position
p0 = [550000 + 6371000; 0; 0];
% Initial velocity
vi = [0; 7597.12185726588*cosd(53); 7597.12185726588*sind(53)];
v0Inertial = [0; 7597.12185726588*cosd(53); 7597.12185726588*sind(53)];
% Inertia tensor in body frame
I_body=[5294.7835 -14.370084 -19.292192;
       -14.370084 5516.4558 -73.354553;
       -19.292192 -73.354553 231.33503];
% Inertia tensor in principal frame
[R_P2B,I_principal]=eig(I_body);
Ixx=I_principal(1,1);
Iyy=I_principal(2,2);
Izz=I_principal(3,3);

R_B2P=R_P2B.';

pointingBody=[0;1;0];
pointingPrincipal=(R_P2B')*pointingBody;
disp(pointingPrincipal)

% Reaction Wheel
IwheelZ=0;
wWheel=0;r=[0;0;0];


%%

% % Uncomment for pset4
theta = 53; % deg rotation along y
q0=quaternion_from_angle(theta);

% Orbit period
T = sqrt((4 * pi^2 * p0(1)^3) / (mu));

% Time step
dt = T/100000;

% Final time
tf = 0.5*T;  


% 3D geometry
geometryBodyFrame = readmatrix('barycenter.xlsx');
geometryPrincipalFrame = zeros(size(geometryBodyFrame));
% process geometry into principal frame
for i = 1:size(geometryBodyFrame, 1)
    currRow = geometryBodyFrame(i,:);
    coordinates = currRow(1:3);
    area = currRow(4);
    unitNormal = currRow(5:7);
    coordPrincipal = R_B2P*coordinates(:);
    unitNormalPrincipal = R_B2P*unitNormal(:);

    geometryPrincipalFrame(i,:) = [coordPrincipal; area; unitNormalPrincipal];
end


%% Pre_processing

% Orbital period

mean_w = 2 *pi/ T;
%Uncomment for P4_4d
w0=[0;0;mean_w];
w_perturb=mean_w*[0.01;0.01;0.01];
w0=w0+w_perturb;
%Uncomment for P4_4e
%w0=[0.01;0.01;0.03];

% Initial Velocity
v0 = principal2Inertial(q0).' * v0Inertial;

% Building the vector to propagate
x = [q0; w0; p0; vi];
% Initializing variables to store
uLog = zeros(6,(ceil(tf/dt))+2);
xLog = zeros(length(x),(ceil(tf/dt))+2);
tLog = 0:dt:(tf+dt);

gravityTorqueLog = zeros(3,(ceil(tf/dt))+2);
magneticTorqueLog= zeros(3,(ceil(tf/dt))+2);
solarTorqueLog= zeros(3,(ceil(tf/dt))+2);
aeroTorqueLog = zeros(3,(ceil(tf/dt))+2);

% Onboard Estimate in quaternion
attitudeErrorLog = zeros(4,(ceil(tf/dt))+2);
attitudeEstimateLog = zeros(4,(ceil(tf/dt))+2);

u = [0; 0; 0; 0; 0; 0];