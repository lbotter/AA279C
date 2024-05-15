true = xLog;

% [q0, q1, q2, q3, wx, wy, wz, px, py, pz, vx, vy, vz]

q0 = true(1,:);
q1 = true(2,:);
q2 = true(3,:);
q3 = true(4,:);
wx = true(5,:);
wy = true(6,:);
wz = true(7,:);
% expressed in ECI (inertial)
px = true(8,:);
py = true(9,:);
pz = true(10,:);
% expressed in principal
vx = true(11,:);
vy = true(12,:);
vz = true(13,:);

ux=uLog(1,:);
uy=uLog(2,:);
uz=uLog(3,:);
% globe visualization
globeRadius = 6371000;
[X,Y,Z] = sphere;
X2 = X * globeRadius;
Y2 = Y * globeRadius;
Z2 = Z * globeRadius;

% DEFINE PARAMETERS HERE --------------------------------------------------
Ixx = 1.0e+03 * 0.230242103134935;
Iyy = 1.0e+03 * 5.293968876196306;
Izz = 1.0e+03 * 5.518363350668759;
I = diag([Ixx, Iyy, Izz]);
principalX = zeros(3, length(q0));
principalY = zeros(3, length(q0));
principalZ = zeros(3, length(q0));
bodyX = zeros(3, length(q0));
bodyY = zeros(3, length(q0));
bodyZ = zeros(3, length(q0));
lvlhX = zeros(3, length(q0));
lvlhY = zeros(3, length(q0));
lvlhZ = zeros(3, length(q0));
vInertial = zeros(3, length(q0));
AngMomPrincipal = zeros(3, length(q0));
AngMomInertial = zeros(3, length(q0));
AngVelInertial = zeros(3, length(q0));
euler_angles=zeros(3,length(q0));
T=zeros(1,length(q0));

for i=1:length(q0)
    q0c = q0(i);
    q1c = q1(i);
    q2c = q2(i);
    q3c = q3(i);

    % principal2inertial rotation matrix
    R_B2I = [q0c^2 + q1c^2 - q2c^2 - q3c^2   2*(q1c*q2c + q0c*q3c)           2*(q1c*q3c - q0c*q2c);
             2*(q1c*q2c - q0c*q3c)           q0c^2 - q1c^2 + q2c^2 - q3c^2   2*(q2c*q3c + q0c*q1c);
             2*(q1c*q3c + q0c*q2c)           2*(q2c*q3c - q0c*q1c)           q0c^2 - q1c^2 - q2c^2 + q3c^2].';

    principalX(:,i) = R_B2I*[1;0;0];
    principalY(:,i) = R_B2I*[0;1;0];
    principalZ(:,i) = R_B2I*[0;0;1];

    vInertial(:,i) = R_B2I*[vx(i); vy(i); vz(i)];

    % angular momentum plotting
    AngMomPrincipal(:,i) = I*[wx(i); wy(i); wz(i)]+ IwheelZ*r*wWheel;
    AngMomInertial(:,i) = R_B2I*I*[wx(i); wy(i); wz(i)] + R_B2I*IwheelZ*r*wWheel;

    % angular velocity inertial
    AngVelInertial(:,i) = R_B2I*[wx(i); wy(i); wz(i)];

    % construct LVLH frame
    lvlhX(:,i) = [px(i); py(i); pz(i)] / norm([px(i); py(i); pz(i)]);
    lvlhZ(:,i) = cross([px(i); py(i); pz(i)], vInertial(:,i)) / norm(cross([px(i); py(i); pz(i)], vInertial(:,i)));
    lvlhY(:,i) = cross(lvlhZ(:,i), lvlhX(:,i));
    
    % construct principal-to-body rotation matrix
    R_P2C = [0.003848270724572  -0.998008358547225   0.062964331826082
             0.013885597875423  -0.062905397652948  -0.997922893372914
             0.999896185103270   0.004714574848179   0.013615865742892].';

    bodyX(:,i) = R_P2C*principalX(:,i);
    bodyY(:,i) = R_P2C*principalY(:,i);
    bodyZ(:,i) = R_P2C*principalZ(:,i);

    % Find Euler angles from quaternion
    euler_angles(:,i) = quat2eul([q0c,q1c,q2c,q3c], 'ZYX');

    T(i)=0.5*[wx(i); wy(i); wz(i)]'*I*[wx(i); wy(i); wz(i)]+ 0.5*IwheelZ*wWheel^2;
    w(i,:)=R_P2C*[wx(i);wy(i);wz(i)];

end

figure; grid on; hold on;
sc = 10000;
quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), principalX(1,1:sc:end), principalX(2,1:sc:end), principalX(3,1:sc:end));
quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), principalY(1,1:sc:end), principalY(2,1:sc:end), principalY(3,1:sc:end));
quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), principalZ(1,1:sc:end), principalZ(2,1:sc:end), principalZ(3,1:sc:end));
% 
% quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), bodyX(1,1:sc:end), bodyX(2,1:sc:end), bodyX(3,1:sc:end));
% quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), bodyY(1,1:sc:end), bodyY(2,1:sc:end), bodyY(3,1:sc:end));
% quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), bodyZ(1,1:sc:end), bodyZ(2,1:sc:end), bodyZ(3,1:sc:end));

% quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), lvlhX(1,1:sc:end), lvlhX(2,1:sc:end), lvlhX(3,1:sc:end), 2.5, AutoScale="off");
% quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), lvlhY(1,1:sc:end), lvlhY(2,1:sc:end), lvlhY(3,1:sc:end), 2.5, AutoScale="off");
% quiver3(px(1:sc:end), py(1:sc:end), pz(1:sc:end), lvlhZ(1,1:sc:end), lvlhZ(2,1:sc:end), lvlhZ(3,1:sc:end), 2.5, AutoScale="off");

figure; grid on; hold on;
title("Angular Momentum: Inertial Frame")
plot(tLog, AngMomInertial(1,:))
plot(tLog, AngMomInertial(2,:))
plot(tLog, AngMomInertial(3,:))
legend("$\vec{H} \cdot \hat{n}_x$", "$\vec{H} \cdot \hat{n}_y$", "$\vec{H} \cdot \hat{n}_z$", 'FontSize', 15, Interpreter="latex")
xlabel("Time (s)")
ylabel("Angular Momentum (kgm^2/s)")
xlim([0,tLog(end)])

figure; grid on; hold on;
sc = 10;
origin = zeros(1, length(tLog));
plot3(AngVelInertial(1,1:sc:end), AngVelInertial(2,1:sc:end), AngVelInertial(3,1:sc:end));
quiver3(origin(1:sc:end), origin(1:sc:end), origin(1:sc:end), 0.0005*AngMomInertial(1,1:sc:end), 0.0005*AngMomInertial(2,1:sc:end), 0.0005*AngMomInertial(3,1:sc:end));
axis equal
legend("Herpolhode", "Angular Momentum Vector")
% 
% figure;
% subplot(3,1,1); grid on; hold on;
% sc = 10;
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), principalX(1,1:sc:end), principalX(2,1:sc:end), principalX(3,1:sc:end));
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), principalY(1,1:sc:end), principalY(2,1:sc:end), principalY(3,1:sc:end));
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), principalZ(1,1:sc:end), principalZ(2,1:sc:end), principalZ(3,1:sc:end));
% axis equal
% legend('Principal X','Principal Y','Principal Z')
% 
% subplot(3,1,2); grid on; hold on;
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), bodyX(1,1:sc:end), bodyX(2,1:sc:end), bodyX(3,1:sc:end));
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), bodyY(1,1:sc:end), bodyY(2,1:sc:end), bodyY(3,1:sc:end));
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), bodyZ(1,1:sc:end), bodyZ(2,1:sc:end), bodyZ(3,1:sc:end));
% axis equal
% legend('Body X','Body Y','Body Z')
% 
% subplot(3,1,3); grid on; hold on;
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), lvlhX(1,1:sc:end), lvlhX(2,1:sc:end), lvlhX(3,1:sc:end));
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), lvlhY(1,1:sc:end), lvlhY(2,1:sc:end), lvlhY(3,1:sc:end));
% quiver3(tLog(1:sc:end), zeros(1,length(tLog(1:sc:end))), zeros(1,length(tLog(1:sc:end))), lvlhZ(1,1:sc:end), lvlhZ(2,1:sc:end), lvlhZ(3,1:sc:end));
% axis equal
% xlabel('Time (s)')
% legend('LVLH X','LVLH Y','LVLH Z')
% linkaxes

figure; grid on; hold on;
title("Quaternion Parameters")
plot(tLog, q0)
plot(tLog, q1)
plot(tLog, q2)
plot(tLog, q3)
xlabel('Time (s)')
legend('q_0', 'q_1', 'q_2', 'q_3')
xlim([0,tLog(end)])

figure; grid on; hold on;
title("Angular velocities")
plot(tLog, wx)
plot(tLog, wy)
plot(tLog, wz)
xlabel('Time (s)')
legend('\omega_x', '\omega_y', '\omega_z')
xlim([0,tLog(end)])

figure; grid on; hold on;
title("Torque")
plot(tLog, ux)
plot(tLog, uy)
plot(tLog, uz)
xlabel('Time (s)')
legend('M_x', 'M_y', 'M_z')
xlim([0,tLog(end)])
ylim([-0.1 0.1])

figure; grid on; hold on;
title("Euler angles ZYX order")
plot(tLog, euler_angles)
xlabel('Time (s)')
legend('\phi', '\theta', '\psi')
xlim([0,tLog(end)])


figure; grid on; hold on;
title("RTN ref frame")
plot(tLog, lvlhX)
plot(tLog, lvlhY)
plot(tLog, lvlhZ)
xlabel('Time (s)')
legend('$R_x$', '$T_y$', '$N_z$')
xlim([0,tLog(end)])


figure; grid on; hold on;
title("Attitude error")
plot(tLog, q0)
plot(tLog, q1)
plot(tLog, q2)
plot(tLog, q3)
xlabel('Time (s)')
legend('q_1', 'q_2', 'q_3','q_4')
xlim([0,tLog(end)])


% figure; grid on; hold on;
% title("Angular Momentum: Principal Frame")
% plot(tLog, AngMomPrincipal(1,:))
% plot(tLog, AngMomPrincipal(2,:))
% plot(tLog, AngMomPrincipal(3,:))
% legend("$\vec{H} \cdot \hat{n}_x$", "$\vec{H} \cdot \hat{n}_y$", "$\vec{H} \cdot \hat{n}_z$", 'FontSize', 15, Interpreter="latex")
% xlabel("Time (s)")
% ylabel("Angular Momentum (kgm^2/s)")
% xlim([0,tLog(end)])
