clear all;
clc;
format long;
close all;

syms dt t m mu ...
     Ixx Ixy Ixz ...
     Iyx Iyy Iyz ...
     Izx Izy Izz ...
     px(t) py(t) pz(t) ...
     phi(t) theta(t) psi(t) ...
     vxI(t) vyI(t) vzI(t) ...   % velocity in inertial frame
     vx(t) vy(t) vz(t) ...
     wx(t) wy(t) wz(t) ...
     q0(t) q1(t) q2(t) q3(t) ...
     tx(t) ty(t) tz(t) ... 
     fx(t) fy(t) fz(t) ... % used for force vector command
     real

Rx = [1,         0,        0;
      0,  cos(phi), sin(phi);
      0, -sin(phi), cos(phi)];
Ry = [cos(theta), 0, -sin(theta);
               0, 1,           0;
      sin(theta), 0,  cos(theta)];
Rz = [cos(psi), sin(psi), 0;
     -sin(psi), cos(psi), 0;
             0,        0, 1];

R_euler = Rx*Ry*Rz;

N = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0,            cos(phi),           -sin(phi);
     0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

I = [Ixx 0   0;
     0   Iyy 0;
     0   0   Izz];

V = [vx(t); vy(t); vz(t)]; % principal frame
W = [wx(t); wy(t); wz(t)]; % principal frame

W_prime = [-q1  q0  q3 -q2;
           -q2 -q3  q0  q1;
           -q3  q2 -q1  q0];

quat = [q0; q1; q2; q3];

R_I2B = [q0^2 + q1^2 - q2^2 - q3^2   2*(q1*q2 + q0*q3)           2*(q1*q3 - q0*q2);
         2*(q1*q2 - q0*q3)           q0^2 - q1^2 + q2^2 - q3^2   2*(q2*q3 + q0*q1);
         2*(q1*q3 + q0*q2)           2*(q2*q3 - q0*q1)           q0^2 - q1^2 - q2^2 + q3^2];

R_B2I = R_I2B.';

H_vehicle = I*W; % total vehicle angular momentum

r = [px; py; pz];
a = [-(mu*px)/( px^2 + py^2 + pz^2)^1.5;
     -(mu*py)/( px^2 + py^2 + pz^2)^1.5;
     -(mu*pz)/( px^2 + py^2 + pz^2)^1.5]; % acceleration in inertial frame

f_principalframe = R_I2B*(m*a) + [fx; fy; fz]; % forces in principal frame
t_principalframe = [tx; ty; tz];               % torques in principal frame

%% f(x) quaternion attitude parameterization
f_continuous_quat = [0.5 * W_prime.' * W;                                % integrate quaternion
                     inv(I) * (t_principalframe - cross(W, H_vehicle));  % wdot (angular accelerations in principal frame)
                     R_B2I * V;                                          % integrate inertial coords R_principal2inertial*V_principal
                    (1/m) * (f_principalframe - cross(W, m*V))];         % vdot (linear accelerations in principal frame)

%% f(x) euler attitude parameterization
f_continuous_euler = [N*W;                                                % integrate euler angles
                      inv(I) * (t_principalframe - cross(W, H_vehicle));  % wdot (angular accelerations)
                      R_euler*V;                                          % integrate inertial coords
                     (1/m) * (f_principalframe - cross(W, m*V))];         % vdot (linear accelerations) 

%%
% Define the inertia tensor
I_body=[5294.7835 -14.370084 -19.292192;
       -14.370084 5516.4558 -73.354553;
       -19.292192 -73.354553 231.33503];

% Eigenvalue analysis
[V,I_principal]=eig(I_body);

%I_body*V-V*I_principal

% Find rotation matrix
xyz_body = eye(3);
xyz_principal = V;

%% Run Simulation
clear all; clc; close all;

dt = 0.01;
tf = 5750;
% tf = 50;

ax = 1; ay = 0; az = 0;
theta = 53; % deg
% theta = 0; % deg

qi = [cosd(theta/2); ax*sind(theta/2); ay*sind(theta/2); az*sind(theta/2)] / norm([cosd(theta/2); ax*sind(theta/2); ay*sind(theta/2); az*sind(theta/2)]);

R_P2B = [0.003848270724572  -0.998008358547225   0.062964331826082
         0.013885597875423  -0.062905397652948  -0.997922893372914
         0.999896185103270   0.004714574848179   0.013615865742892].';

quat_P2B = DCMtoQuaternion(R_P2B);


% wi = [0; 0; 4*(2*pi) / 5750];
wi = [0; 0; 0];
pi = [550000 + 6371000; 0; 0];
viInertial = [0; 7597.12185726588*cosd(53); 7597.12185726588*sind(53)];

q0 = qi(1);
q1 = qi(2);
q2 = qi(3);
q3 = qi(4);

% principal2inertial rotation matrix
R_I2B = [q0^2 + q1^2 - q2^2 - q3^2   2*(q1*q2 + q0*q3)           2*(q1*q3 - q0*q2);
         2*(q1*q2 - q0*q3)           q0^2 - q1^2 + q2^2 - q3^2   2*(q2*q3 + q0*q1);
         2*(q1*q3 + q0*q2)           2*(q2*q3 - q0*q1)           q0^2 - q1^2 - q2^2 + q3^2];
vi = R_I2B*viInertial;
x = [qi; wi; pi; vi];

uLog = zeros(6,(tf/dt)+2);
xLog = zeros(length(x),(tf/dt)+2);
tLog = 0:dt:(tf+dt);

for k = 0:length(0:dt:tf)
    u = [0; 0; 0; 0; 0; 0];
    % x(8:10) = [0;0;0];
    xNew = fDiscreteRK4(x, u, dt);

    uLog(:,k+1) = u;
    xLog(:,k+1) = x;

    x = xNew;
end

plotResults;

%% functions
function xNew = fDiscreteRK4(x, u, dt)
    % Performs a forward RK4 step assuming a zero-order-held u
    k1 = fContinuous(x, u);
    k2 = fContinuous(x + dt*k1/2, u);
    k3 = fContinuous(x + dt*k2/2, u);
    k4 = fContinuous(x + dt*k3, u);
    
    xNew = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    xNew(1:4) = xNew(1:4)./norm(xNew(1:4)); % normalize quaternion
end

function xDot = fContinuous(x, u)
    % Returns the nonlinear continuous-time derivative, x_dot = f(x, u)
    
    % Inputs:
    % x: state, [q0; q1; q2; q3; wx; wy; wz]
    % u: input, [tx; ty; tz]
    
    % Outputs:
    % xDot

    q0 = x(1);
    q1 = x(2);
    q2 = x(3);
    q3 = x(4);
    wx = x(5);
    wy = x(6);
    wz = x(7);
    px = x(8);
    py = x(9);
    pz = x(10);
    vx = x(11);
    vy = x(12);
    vz = x(13);

    tx = u(1);
    ty = u(2);
    tz = u(3);
    fx = u(4);
    fy = u(5);
    fz = u(6);

    % parameters
    Ixx = 1.0e+03 * 0.230242103134935;
    Iyy = 1.0e+03 * 5.293968876196306;
    Izz = 1.0e+03 * 5.518363350668759;
    m = 260;
    mu = 3.986004418e14;

    xDot = [-0.5000*q1*wx - 0.5000*q2*wy - 0.5000*q3*wz;
             0.5000*q0*wx - 0.5000*q3*wy + 0.5000*q2*wz;
             0.5000*q3*wx + 0.5000*q0*wy - 0.5000*q1*wz;
             0.5000*q1*wy - 0.5000*q2*wx + 0.5000*q0*wz;
            (tx + Iyy*wy*wz - Izz*wy*wz)/Ixx;
            (ty - Ixx*wx*wz + Izz*wx*wz)/Iyy;
            (tz + Ixx*wx*wy - Iyy*wx*wy)/Izz;
             vx*(q0^2 + q1^2 - q2^2 - q3^2) - vy*(2*q0*q3 - 2*q1*q2) + vz*(2*q0*q2 + 2*q1*q3);
             vy*(q0^2 - q1^2 + q2^2 - q3^2) + vx*(2*q0*q3 + 2*q1*q2) - vz*(2*q0*q1 - 2*q2*q3);
             vz*(q0^2 - q1^2 - q2^2 + q3^2) - vx*(2*q0*q2 - 2*q1*q3) + vy*(2*q0*q1 + 2*q2*q3);
            (fx + m*vy*wz - m*vz*wy - (m*mu*px*(q0^2 + q1^2 - q2^2 - q3^2))/(px^2 + py^2 + pz^2)^1.5 - (m*mu*py*(2*q0*q3 + 2*q1*q2))/(px^2 + py^2 + pz^2)^1.5 + (m*mu*pz*(2*q0*q2 - 2*q1*q3))/(px^2 + py^2 + pz^2)^1.5)/m;
            (fy - m*vx*wz + m*vz*wx - (m*mu*py*(q0^2 - q1^2 + q2^2 - q3^2))/(px^2 + py^2 + pz^2)^1.5 + (m*mu*px*(2*q0*q3 - 2*q1*q2))/(px^2 + py^2 + pz^2)^1.5 - (m*mu*pz*(2*q0*q1 + 2*q2*q3))/(px^2 + py^2 + pz^2)^1.5)/m;
            (fz + m*vx*wy - m*vy*wx - (m*mu*pz*(q0^2 - q1^2 - q2^2 + q3^2))/(px^2 + py^2 + pz^2)^1.5 - (m*mu*px*(2*q0*q2 + 2*q1*q3))/(px^2 + py^2 + pz^2)^1.5 + (m*mu*py*(2*q0*q1 - 2*q2*q3))/(px^2 + py^2 + pz^2)^1.5)/m];

end

