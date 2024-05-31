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
tf = 0.3*T;  


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

%% Pre_processing

% Orbital period

T = sqrt((4 * pi^2 * p0(1)^3) / (mu));
mean_w = 2 *pi/ T;
%Uncomment for P4_4d
%w0=[0;0;mean_w];
%Uncomment for P4_4e
w0 = R_P2B.' * [0.01; 0.01; 0.01];

% Building the vector to propagate
x = [q0; w0; p0; v0];
mean = [q0; w0];
meanPredict = [q0; w0];
cov = 0.01*eye(length(mean));
Q = 0.01*dt*diag([1, 1, 1, 1, 1, 1, 1]);
R =      dt*diag([deg2rad(0.1)^2, deg2rad(0.1)^2, deg2rad(0.1)^2, ... % gyro
                  deg2rad(0.01)^2, deg2rad(0.01)^2, deg2rad(0.01)^2, ... % sun sensor
                  5e-9, 5e-9, 5e-9]);                                    % mag sensor

% Initializing variables to store

gravityTorqueLog = zeros(3,(tf/dt)+2);
magneticTorqueLog = zeros(3,(tf/dt)+2);
solarTorqueLog = zeros(3,(tf/dt)+2);
aeroTorqueLog = zeros(3,(tf/dt)+2);
aeroForceLog = zeros(3,(tf/dt)+2);

uLog = zeros(6,(tf/dt)+2);
xLog = zeros(length(x),(tf/dt)+2);
yLog = zeros(9,(tf/dt)+2);
meanLog = zeros(length(mean),(tf/dt)+2);
meanPredictLog = zeros(length(mean),(tf/dt)+2);
varianceLog = zeros(length(mean),(tf/dt)+2);
tLog = 0:dt:(tf+dt);
u = [0; 0; 0; 0; 0; 0];