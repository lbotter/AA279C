%% Functions
function out = skew(x)
a = x(1);
b = x(2);
c = x(3);

out = [0  -c   b;
       c   0  -a;
      -b   a   0];
end

function quat = DCMtoQuaternion(DCM)
% FUNCTION NAME:
%   DCMtoQuaternion
%
% DESCRIPTION:
%    This function converts a rotation represented in directional cosine 
%    matrix form to a rotation represented in unit quaternion form.
%
% INPUTS:
%   DCM       double   3x3   Input directional cosine matrix.
%
% OUTPUTS:
%   q         double   4x1   Output quaternion corresponding to the rotation represented by the input DCM.

quat = zeros(4, 1);
u = reshape(DCM, 9, 1); 
% u = DCM flattened

trace = u(1) + u(5) + u(9);
if trace > 0
    z =sqrt(trace+1);
    b = [u(8)-u(6), u(3)-u(7), u(4)-u(2)]./(z*2);
    quat = [0.5*z, b(1), b(2), b(3)]';
else
    if u(5) > u(1) && u(5) > u(9)
        s = sqrt(u(5) - (u(1)+u(9)) + 1);
        a = 0;
        if s~=0; a = 0.5/s; end
        b = a.*[u(4)+u(2), u(8)+u(6), u(3)-u(7)];
        quat = [b(3), b(1), 0.5*s, b(2)]';
    elseif u(9) > u(1)
        s = sqrt(u(9) - (u(1)+u(5)) + 1);
        a = 0;
        if s~=0; a = 0.5/s; end
        b = a.*[u(3)+u(7), u(8)+u(6), u(4)-u(2)];
        quat = [b(3), b(1), b(2), 0.5*s]';
    else
        s = sqrt(u(1) - (u(5)+u(9)) + 1);
        a = 0;
        if s~=0; a = 0.5/s; end
        b = a.*[u(4)+u(2), u(3)+u(7), u(8)-u(6)];
        quat = [b(3), 0.5*s, b(1), b(2)]';
    end
end

quat = quat ./ norm(quat);

end

function q = quaternionMultiply(q1, q2)
    %{
        Function: quaternionMultiply.m
        Input(s): 
            1) q1: First quaternion - [4x1]
            2) q2: Second quaternion - [4x1]
        Output(2):
            1) q: Resultant quaternion - [4x1]
        Usage: Perform multiplication of the two input quaternions
                q = q1*q2
        Reference: https://www.mathworks.com/help/aerotbx/ug/quatmultiply.html
    %}
    q1 = q1(:);
    q2 = q2(:);
    q = [q1(1), -q1(2:4)';
        q1(2:4), q1(1)*eye(3) + skew(q1(2:4))] * q2;

    if q(1) < 0
        q = -1*q;
    end
end

function qInv = quaternionInverse(q)
    %{
        Function: quaternionInverse.m
        Input(s): 
            1) q: quaternion - [4x1]
        Output(2):
            1) qInv: Inverse quaternion - [4x1]
        Usage: Compute inverse of the input quaternion
                qInv = q^(-1)
        Reference: https://www.mathworks.com/help/aerotbx/ug/quatinv.html
    %}
    
    qInv = [q(1); -q(2); -q(3); -q(4)];
    qInv = qInv ./ norm(qInv);
end


% Pre processing
true = xLog;
est = meanLog;

% [q0, q1, q2, q3, wx, wy, wz, px, py, pz, vx, vy, vz]

q0 = true(1,:);
q1 = true(2,:);
q2 = true(3,:);
q3 = true(4,:);
wx = true(5,:);
wy = true(6,:);
wz = true(7,:);
px = true(8,:);
py = true(9,:);
pz = true(10,:);
vx = true(11,:);
vy = true(12,:);
vz = true(13,:);

q0Est = est(1,:);
q1Est = est(2,:);
q2Est = est(3,:);
q3Est = est(4,:);
wxEst = est(5,:);
wyEst = est(6,:);
wzEst = est(7,:);

% globe visualization
globeRadius = 6371000;
[X,Y,Z] = sphere;
X2 = X * globeRadius;
Y2 = Y * globeRadius;
Z2 = Z * globeRadius;

%%
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
AngVelLVLH = zeros(3, length(q0));
quatLVLH = zeros(4, length(q0));
eulerPhiThetaPsi = zeros(3, length(q0));
eulerPhiThetaPsiEst = zeros(3, length(q0));
eulerAngleError = zeros(3, length(q0));
eulerPhiThetaPsiLVLH = zeros(3, length(q0));
quaternionError=zeros(4, length(q0));
angvelerror=zeros(3,length(q0));

varianceEul=zeros(3, length(q0));

stateEstPropogationError = zeros(1, length(q0));

orbitRadius = zeros(1, length(q0));

for i=1:length(q0)

    orbitRadius(:,i) = norm([px(i), py(i), pz(i)]);
    
    stateEstPropogationError(i) = norm(meanLog(:,i) - [q0(i); q1(i); q2(i); q3(i); wx(i); wy(i); wz(i)]);

    q0c = q0(i);
    q1c = q1(i);
    q2c = q2(i);
    q3c = q3(i);

    eulerPhiThetaPsi(:,i) = quat2Eul([q0(i); q1(i); q2(i); q3(i)]);
    eulerPhiThetaPsiEst(:,i) = quat2Eul([q0Est(i); q1Est(i); q2Est(i); q3Est(i)]);
    eulerAngleError(:,i) = abs( eulerPhiThetaPsi(:,i) - eulerPhiThetaPsiEst(:,i) );
    varianceEul(:,i)=quat2Eul(varianceLog(1:4,i));



    quaternionError(:,i)=attitudeError([q0c q1c q2c q3c],meanLog(1:4,i));
    angvelerror(:,i)=abs(meanLog(5:7,i)-[wx(i);wy(i);wz(i)]);

    if any( eulerAngleError(1,i) > deg2rad(300) )
        eulerAngleError(1,i) = abs(2*pi - eulerAngleError(1,i));
    end

    if any( eulerAngleError(2,i) > deg2rad(300) )
        eulerAngleError(2,i) = abs(2*pi - eulerAngleError(2,i));
    end

    if any( eulerAngleError(3,i) > deg2rad(300) )
        eulerAngleError(3,i) = abs(2*pi - eulerAngleError(3,i));
    end

    % principal2inertial rotation matrix
    R_B2I = [q0c^2 + q1c^2 - q2c^2 - q3c^2   2*(q1c*q2c + q0c*q3c)           2*(q1c*q3c - q0c*q2c);
             2*(q1c*q2c - q0c*q3c)           q0c^2 - q1c^2 + q2c^2 - q3c^2   2*(q2c*q3c + q0c*q1c);
             2*(q1c*q3c + q0c*q2c)           2*(q2c*q3c - q0c*q1c)           q0c^2 - q1c^2 - q2c^2 + q3c^2].';

    principalX(:,i) = R_B2I*[1;0;0];
    principalY(:,i) = R_B2I*[0;1;0];
    principalZ(:,i) = R_B2I*[0;0;1];

    vInertial(:,i) = R_B2I*[vx(i); vy(i); vz(i)];

    % angular momentum plotting
    AngMomPrincipal(:,i) = I*[wx(i); wy(i); wz(i)];
    AngMomInertial(:,i) = R_B2I*I*[wx(i); wy(i); wz(i)];

    % angular velocity inertial
    AngVelInertial(:,i) = R_B2I*[wx(i); wy(i); wz(i)];

    % construct LVLH frame
    lvlhX(:,i) = [px(i); py(i); pz(i)] / norm([px(i); py(i); pz(i)]);
    lvlhZ(:,i) = cross([px(i); py(i); pz(i)], vInertial(:,i)) / norm(cross([px(i); py(i); pz(i)], vInertial(:,i)));
    lvlhY(:,i) = cross(lvlhZ(:,i), lvlhX(:,i));

    % construct inertial-to-LVLH rotation matrix
    R_I2L = [lvlhX(:,i), lvlhY(:,i), lvlhZ(:,i)].';

    % angular velocity LVLH
    AngVelLVLH(:,i) = R_I2L*AngVelInertial(:,i);

    quat_I2L = dcm2quat(R_I2L);
    quatLVLH(:,i) = quaternionMultiply(quaternionInverse(quat_I2L), [q0c;q1c;q2c;q3c]);

    q0c = quatLVLH(1,i);
    q1c = quatLVLH(2,i);
    q2c = quatLVLH(3,i);
    q3c = quatLVLH(4,i);

    eulerPhiThetaPsiLVLH(:,i) = [atan2(2*(q0c*q1c + q2c*q3c), 1-2*(q1c^2 + q2c^2));
                                -3.14159265/2 + 2*atan2(sqrt(1 + 2*(q0c*q2c - q1c*q3c)), sqrt(1 - 2*(q0c*q2c - q1c*q3c)));
                                 atan2(2*(q0c*q3c + q1c*q2c), 1-2*(q2c^2 + q3c^2))];

    % construct principal-to-body rotation matrix
    R_P2B = [0.003848270724572  -0.998008358547225   0.062964331826082
             0.013885597875423  -0.062905397652948  -0.997922893372914
             0.999896185103270   0.004714574848179   0.013615865742892].';

    bodyX(:,i) = R_P2B*principalX(:,i);
    bodyY(:,i) = R_P2B*principalY(:,i);
    bodyZ(:,i) = R_P2B*principalZ(:,i);

end

% Show confidence intervals using standard deviation
showConfidence = @(t, data, std_dev) fill([t, fliplr(t)], ...
                                         [data + 2*std_dev, fliplr(data - 2*std_dev)], ...
                                         'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
stLog=sqrt(varianceLog);

%%

% Variance
figure; grid on; hold on;
plot(tLog, varianceLog)
title("Filter Variances Over Time", 'FontSize', 15, Interpreter="latex")
legend('$q_0$', '$q_1$', '$q_2$', '$q_3$', "$\omega_x$", "$\omega_y$", "$\omega_z$", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
xlim([0,tLog(end)])

% Euler angles error
figure; grid on; hold on;
plot(tLog, rad2deg(eulerAngleError))
title("Attitude Estimate Error (Euler Angles)", 'FontSize', 15, Interpreter="latex")
legend("$\phi$ Err.", "$\theta$ Err.", "$\psi$ Err.", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Angle (deg)')
xlim([0,tLog(end)])

%% 

% Show confidence intervals using standard deviation
showConfidence = @(t, data, std_dev) fill([t, fliplr(t)], ...
                                         [data + 2*std_dev, fliplr(data - 2*std_dev)], ...
                                         'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Quaternions error
figure; grid on; hold on;
plot(tLog, quaternionError)
title("Attitude Estimate Error (Quaternions)", 'FontSize', 15, Interpreter="latex")
legend("$q_0$ Err.", "$q_1$ Err.", "$q_2$ Err.","$q_3$ Err", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Quaternion')
xlim([0,tLog(end)])


% Average
figure; grid on; hold on;
plot(tLog, mean(quaternionError,1))
title("Attitude Estimate Error avg(Quaternions)", 'FontSize', 15, Interpreter="latex")
legend("avg error", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Quaternion')
xlim([0,tLog(end)])

figure; grid on; hold on;
title("Quaternion Parameters Ground Truth", 'FontSize', 15, Interpreter="latex")
plot(tLog, q0)
plot(tLog, q1)
plot(tLog, q2)
plot(tLog, q3)
xlabel('Time (s)')
legend('$q_0$', '$q_1$', '$q_2$', '$q_3$', 'FontSize', 15, Interpreter="latex")
xlim([0,tLog(end)])


figure; grid on; hold on;
title("Quaternion Parameters Estimate with std", 'FontSize', 15, Interpreter="latex")
plot(tLog, q0Est)
plot(tLog, q1Est)
plot(tLog, q2Est)
plot(tLog, q3Est)
showConfidence(tLog,q0Est,stLog(1,:));
showConfidence(tLog,q1Est,stLog(2,:));
showConfidence(tLog,q2Est,stLog(3,:));
showConfidence(tLog,q3Est,stLog(4,:));
xlabel('Time (s)')
legend('$q_0$ Est.', '$q_1$ Est.', '$q_2$ Est.', '$q_3$ Est.', 'FontSize', 15, Interpreter="latex")
xlim([0,tLog(end)])
%%

index=tLog>200;
mean_error=mean(rad2deg((quaternionError(:,index))),2)
std_error=std(rad2deg(quaternionError(:,index)),0,2)


%%

% Euler angles
stEul=10*sqrt(rad2deg(sqrt(varianceEul)));

% mean euler angle error
clear mean
figure; grid on; hold on;
plot(tLog, rad2deg(mean(eulerAngleError,1)))
title("Attitude Estimate Error (Euler Angles)", 'FontSize', 15, Interpreter="latex")
legend("Avg total error", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Angle (deg)')
xlim([0,tLog(end)])



figure; grid on; hold on;
plot(tLog, rad2deg(eulerPhiThetaPsi))
title("Euler Angles (ECI)")
legend("$\phi$", "$\theta$", "$\psi$", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Angle (deg)')
xlim([0,tLog(end)])

figure; grid on; hold on;
plot(tLog, rad2deg(eulerPhiThetaPsiEst))
title("Euler Angles Estimate (ECI)")
hold on
showConfidence(tLog,rad2deg(eulerPhiThetaPsiEst(1,:)),stEul(1,:));
showConfidence(tLog,rad2deg(eulerPhiThetaPsiEst(2,:)),stEul(2,:));
showConfidence(tLog,rad2deg(eulerPhiThetaPsiEst(3,:)),stEul(3,:));
legend("$\phi$ Est.", "$\theta$ Est.", "$\psi$ Est.", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Angle (deg)')
xlim([0,tLog(end)])

% Estimate error after settling time
index=tLog>200;
mean_error=mean(rad2deg((eulerAngleError(:,index))),2)
std_error=std(rad2deg(eulerAngleError(:,index)),0,2)



%%

% Angular velocities

figure; grid on; hold on;
plot(tLog, [wx;wy;wz])
title("Angular Velocities In Principal Frame")
legend("$\omega_x$", "$\omega_y$", "$\omega_z$", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Angular Rate (rad/s)')
xlim([0,tLog(end)])


figure; grid on; hold on;
plot(tLog, [wxEst;wyEst;wzEst])
showConfidence(tLog,wxEst,stLog(5,:));
showConfidence(tLog,wyEst,stLog(6,:));
showConfidence(tLog,wzEst,stLog(7,:));
title("Angular Velocities In Principal Frame")
legend("$\omega_x$ Est.", "$\omega_y$ Est.", "$\omega_z$ Est.", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Angular Rate (rad/s)')
xlim([0,tLog(end)])
ylim([-0.02,0.02])

% error
figure; grid on; hold on;
plot(tLog,angvelerror)
title("Angular velocities estimate error", 'FontSize', 15, Interpreter="latex")
legend("$w_x$ Err.", "$w_y$ Err.", "$w_z$ Err.", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Quaternion')
xlim([0,tLog(end)])


% Average
figure; grid on; hold on;
plot(tLog, mean(angvelerror,1))
title("Angular velocities RMSE", 'FontSize', 15, Interpreter="latex")
legend("avg error", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('rad/s')
xlim([0,tLog(end)])



%%
index=tLog>200;
mean_error=mean(rad2deg((angvelerror(:,index))),2)
std_error=std(rad2deg(eulerAngleError(:,index)),0,2)


%%
% Residuals

smoothedDataPre = smooth(tLog, vecnorm(preFitLog(7:9,:)), 0.01, 'lowess'); % Adjust 0.1 for more or less smoothing
subplot(2,1,1)
plot(tLog, vecnorm(preFitLog(7:9,:),2),'+b');
hold on
plot(tLog,smoothedDataPre,'r');
title("Pre fit residuals norm for magnetometer")
legend("pre fit","smoothed data", Interpreter="latex")
xlabel('Time (s)')
ylabel('Residual')
xlim([0,tLog(end)])
grid on

% Residuals
windowSize=100;
smoothedDataPost = smooth(tLog, vecnorm(postFitLog(7:9,:)), 0.01, 'lowess'); % Adjust 0.1 for more or less smoothing
subplot(2,1,2)
plot(tLog, vecnorm(postFitLog(7:9,:),2),'+b');
hold on
plot(tLog,smoothedDataPost,'r');
title("Post fit residuals norm for magnetometer")
legend("post fit","smoothed data", Interpreter="latex")
xlabel('Time (s)')
ylabel('Residual')
xlim([0,tLog(end)])
grid on

%%
index=tLog>200;
mean_error_pre=mean(preFitLog(1:3,:),2)
std_error_pre=std(preFitLog(1:3,:),0,2)

index=tLog>200;
mean_error_post=mean(postFitLog(1:3,:),2)
std_error_post=std(postFitLog(1:3,:),0,2)





%%

figure; grid on; hold on;
plot(tLog, stateEstPropogationError)
title("State Estimator Propogation Error (2-norm Across All Attitude States)", 'FontSize', 15, Interpreter="latex")
xlabel('Time (s)')
ylabel('Magnitude')
xlim([0,tLog(end)])

figure; grid on; hold on;

% Show confidence intervals using standard deviation
showConfidence = @(t, data, std_dev) fill([t, fliplr(t)], ...
                                         [data + 2*std_dev, fliplr(data - 2*std_dev)], ...
                                         'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
stdLog=sqrt(varianceLog);
% Plot the confidence interval for q0Est
showConfidence(tLog, q0Est, stdLog(1,:))

% Plot the estimate for q0Est
q0Plot = plot(tLog, q0Est);
legend()

% Add labels and legend
xlabel('Time');
ylabel('q0 Estimate');
legend;
title('q0 Estimate with Confidence Interval');

% Calculate and plot statistics at steady state (assume steady state after time > 50)
steady_state_indices = find(tLog > 500);
mean_errors = mean(eulerAngleError(:, steady_state_indices), 2);
std_errors = std(eulerAngleError(:, steady_state_indices), 0, 2);

figure;
errorbar([1,2,3], mean_errors, std_errors, 'o');
title('True Attitude Estimation Errors Statistics at Steady State');
xlabel('Axis');
ylabel('Error');
xticks([1 2 3]);
xticklabels({'X', 'Y', 'Z'});
