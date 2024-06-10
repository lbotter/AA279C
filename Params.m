% Parameters definition

%% TIME PARAMETERS
% Time step
dt = 0.01; 

% Final time 
tf = 300; 
%% SATELLITE GEOMETRY PARAMETERS
% Satellite Mass
m = 260;
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

earthPointingVec=[0;0;1]; % z axis points the earth 
trackPointingVec=[0;1;0]; % y axis points the track

% PD controller parameters
zeta = 0.707;
nFreq = 0.05;

%% Reaction Wheel Parameters
rWheel.torqueLim = 1; % Nm
rWheel.m = 5;
rWheel.r2 = 0.1524;
rWheel.r1 = 0.1375;
rWheel.Iz = (1/2)*rWheel.m*(rWheel.r2^2 + rWheel.r1^2);
rWheel.dw = 0.04; %rad/s max dW for desaturation
rWheel.wmax=400;
rWheel.matrix=1/sqrt(3) * [-1  1  1 -1;
                                  -1 -1  1  1;
                                   1  1  1  1];
rWheel.imatrix=3/(4*sqrt(3)) * [-1 -1  1;
                                          1 -1  1;
                                          1  1  1;
                                         -1  1  1];



%% Thrusters
%capabilities
Thruster.Tmax=170e-2;   % Newton
Thruster.Tmin=20e-3;
Thruster.tburn=2000; % Total burn time for each thruster

% Geometry

% Number of thrusters
N=4;
% top left
r(:,1)=[0.05;-2.7892;1.6];
e(:,1)=-[0.5;-1;0.5]/norm([0.5;-1;0.5]);
% top right
r(:,2)=[0.05;-2.7892;-1.6];
e(:,2)=-[0.5;-1;-0.5]/norm([0.5;-1;0.5]);
% bottom left
r(:,3)=[-0.05;-2.7892;1.6];
e(:,3)=-[-0.5;-1;0.5]/norm([0.5;-1;0.5]);
% bottom right
r(:,4)=[-0.05;-2.7892;-1.6];
e(:,4)=-[-0.5;-1;-0.5]/norm([0.5;-1;0.5]);


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

% PD controller 
kd = 2*zeta*nFreq*[Ixx;
                   Iyy;
                   Izz];
      
kp = (nFreq^2)*[Ixx;
                Iyy;
                Izz];
% Thrust matrix
Thruster.A=zeros(3,N);
for i=1:N
    Thruster.A(:,i)=cross(r(:,i),e(:,i));
end
Thruster.E=e;


% Building the vectors to propagate
x = [q0; w0; p0; v0];
deltaU=[0;0;0];
meancomp = [q0; w0];
meanPredict = [q0; w0];
cov = 0.01*eye(length(meancomp));
reactionWheelTorque = [0;0;0];
desatTime=0;
wsOut=0;
uThrusters=zeros(6,1);

% Initializing variables to store
L=tf/dt+2;
gravityTorqueMagLog     = zeros(1,L);
magneticTorqueMagLog    = zeros(1,L);
solarTorqueMagLog       = zeros(1,L);
aeroTorqueMagLog        = zeros(1,L);
aeroForceMagLog         = zeros(1,L);

gravityTorqueLog        = zeros(3,L);
magneticTorqueLog       = zeros(3,L);
solarTorqueLog          = zeros(3,L);
aeroTorqueLog           = zeros(3,L);
aeroForceLog            = zeros(3,L);
disturbLog              = zeros(3,L);

uLog = zeros(6,L);
xLog = zeros(length(x),L);
yLog = zeros(9,L);
tLog = 0:dt:(tf+dt);

deltaULog = zeros(3,L);
reactionWheelTorqueLog = zeros(3,L);
reactionWheelSpeedLog = zeros(4,L);
desiredAttitudeLog = zeros(4,L);

meanLog         = zeros(length(meancomp),L);
meanPredictLog  = zeros(length(meancomp),L);
varianceLog     = zeros(length(meancomp),L);
preFitLog       = zeros(9,L);
postFitLog      = zeros(9,L);
thrustLog       = zeros(N,L);
thrustPushLog   = zeros(6,L);
a=[0;0;0];