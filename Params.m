% Parameters definition

%% TIME PARAMETERS
% Time step
dt = 0.01; 

% Final time 
tf = 100; 
%% SATELLITE GEOMETRY PARAMETERS
% Satellite Mass
m = 260;
% Momentum wheel parameters
IwheelZ = 5;  % kg*m^2
wWheel = 100; % rad/s
% Inertia tensor in body frame
I_body=[5294.7835 -14.370084 -19.292192;
       -14.370084 5516.4558 -73.354553;
       -19.292192 -73.354553 231.33503];

% Inertia tensor in principal frame
[R_P2B, I_principal] = eig(I_body);
Ixx = I_principal(1,1);
Iyy = I_principal(2,2);
Izz = I_principal(3,3);

% 3D geometry
geometryBodyFrame = readmatrix('barycenter.xlsx');
geometryPrincipalFrame = zeros(size(geometryBodyFrame));

% Spinning wheel axis
r_body = [0;1;0];
r_principal=R_P2B*r_body;
R_B2P = R_P2B.';
%% ORBITAL PARAMETERS

% Initial forces acting to the body
u = [0; 0; 0; 0; 0; 0];
% Cavendish constant for Earth
mu = 3.986004418e14;
theta = 53; % deg rotation around x
q0 = angle2Quaternion(theta);
q0=[0.6328;
    0.3155;
   -0.6328;
   -0.3155];
%Initial position
p0 = [550000 + 6371000; 0; 0];
% Initial velocity
v0Inertial = [0; 7597.12185726588*cosd(53); 7597.12185726588*sind(53)];
v0 = principal2Inertial(q0).' * v0Inertial;

%% SATELLITE ATTITUDE POSITIONING

% Vectors in principal frame that need to point the earth or the direction
% of motion

earthPointingVec=[0;0;1]; 
trackPointingVec=[0;1;0];

responseF=100;

%% EKF PARAMETERS

Q = 0.01*dt*diag([1, 1, 1, 1, 1, 1, 1]);
R =      dt*diag([deg2rad(0.1)^2, deg2rad(0.1)^2, deg2rad(0.1)^2, ... % gyro
                  deg2rad(0.01)^2, deg2rad(0.01)^2, deg2rad(0.01)^2, ... % sun sensor
                  5e-9, 5e-9, 5e-9]);                                    % mag sensor
%% PRE PROCESSING


% Process Geometry into principal frame
for i = 1:size(geometryBodyFrame, 1)
    currRow = geometryBodyFrame(i,:);
    coordinates = currRow(1:3);
    area = currRow(4);
    unitNormal = currRow(5:7);

    coordPrincipal = R_B2P*coordinates(:);
    unitNormalPrincipal = R_B2P*unitNormal(:);

    geometryPrincipalFrame(i,:) = [coordPrincipal; area; unitNormalPrincipal];
end

% Orbital period for mean motion
T = sqrt((4 * pi^2 * p0(1)^3) / (mu));
mean_w = 2 *pi/ T;
w0 = R_P2B.' * [0.01; 0.01; 0.01];
n=mean_w;

% PD Controller
kp=responseF^2/Izz;
kd=2*sqrt(Izz*(3*n^2*(Iyy-Ixx)+kp));

% Building the vectors to propagate
x = [q0; w0; p0; v0];
deltaU=[0;0;0];
meancomp = [q0; w0];
meanPredict = [q0; w0];
cov = 0.01*eye(length(meancomp));
reactionWheelTorque = [0;0;0];

% Initializing variables to store
gravityTorqueMagLog     = zeros(1,(tf/dt)+2);
magneticTorqueMagLog    = zeros(1,(tf/dt)+2);
solarTorqueMagLog       = zeros(1,(tf/dt)+2);
aeroTorqueMagLog        = zeros(1,(tf/dt)+2);
aeroForceMagLog         = zeros(1,(tf/dt)+2);

gravityTorqueLog        = zeros(3,(tf/dt)+2);
magneticTorqueLog       = zeros(3,(tf/dt)+2);
solarTorqueLog          = zeros(3,(tf/dt)+2);
aeroTorqueLog           = zeros(3,(tf/dt)+2);
aeroForceLog            = zeros(3,(tf/dt)+2);

uLog = zeros(6,(tf/dt)+2);
xLog = zeros(length(x),(tf/dt)+2);
yLog = zeros(9,(tf/dt)+2);
tLog = 0:dt:(tf+dt);
deltaULog = zeros(3,(tf/dt)+2);
reactionWheelTorqueLog = zeros(3,(tf/dt)+2);
desiredAttitudeLog = zeros(4,(tf/dt)+2);

meanLog         = zeros(length(meancomp),(tf/dt)+2);
meanPredictLog  = zeros(length(meancomp),(tf/dt)+2);
varianceLog     = zeros(length(meancomp),(tf/dt)+2);
preFitLog       = zeros(9,(tf/dt)+2);
postFitLog      = zeros(9,(tf/dt)+2);
a=[0;0;0];