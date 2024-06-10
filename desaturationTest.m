clear;
clc;
close all;
Params;
span=800;
deltaU=[0;0;1];
tLog=zeros(span,1);
thrustLog=zeros(4,span);
thrustPushLog=zeros(6,span); 
reactionWheelTorqueLog=zeros(3,span);
reactionWheelSpeedLog=zeros(4,span);
deltaULog=zeros(3,span);
thrustInputLog=zeros(6,span);
uLog=zeros(6,span);
rWheel.wmax=10;
desatTime=0;
wsOut=0;

[reactionWheelTorque, wsOut] = reactionWheelActuator(x, deltaU, rWheel, dt,true);
for k=0:(span-1)
    if max(abs(wsOut))<rWheel.wmax && desatTime==0
        [reactionWheelTorque, wsOut] = reactionWheelActuator(x, deltaU, rWheel, dt,false);
        uThrustInput=zeros(6,1);
        [uThrusters,T]=thrusterActuator(uThrustInput,Thruster,1);
    else
        [uThrustInput,desatTime] = reactionWheelDesaturation(rWheel,wsOut,dt,desatTime);
        [reactionWheelTorque, wsOut] = reactionWheelActuator(x,- uThrustInput(1:3), rWheel, dt,false);
        uThrustInput(1:3)=uThrustInput(1:3)+deltaU;
        [uThrusters,T]=thrusterActuator(uThrustInput,Thruster,1);
    end
    u(1:3)=reactionWheelTorque+ uThrusters(1:3);


    deltaULog(:,k+1) = deltaU;
    reactionWheelTorqueLog(:,k+1) = reactionWheelTorque;
    reactionWheelSpeedLog(:,k+1) = wsOut;
    thrustLog(:,k+1)=T;
    thrustInputLog(:,k+1)=uThrustInput;
    thrustPushLog(:,k+1)=uThrusters;
    uLog(:,k+1) = u;
    tLog(k+1)=k;
end
%%
%% Plot

clear ylim
clear xlim
figure(1)
subplot(3,1,1)
grid on; hold on;
plot(tLog,thrustPushLog(1:3,:))
title("Torques", 'FontSize', 15, Interpreter="latex")
legend("$M_x$ ", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])


subplot(3,1,2)
grid on; hold on;
plot(tLog,thrustPushLog(4:6,:))
title("Forces", 'FontSize', 15, Interpreter="latex")
legend("$F_{x}$ ", "$F_{y}$", "$F_{z}$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Forces (N)')

subplot(3,1,3)
grid on; 
hold on;
plot(tLog, thrustLog, 'DisplayName', 'Thrust');
title('t Log', 'FontSize', 15, 'Interpreter', 'latex');
legend('$t_1$', '$t_2$', '$t_3$', '$t_4$', 'Interpreter', 'latex');
% Set x and y labels
xlabel('Time (s)');
ylabel('Thrust (N)');
% Set x and y axis limits
xlim([0, tLog(end)]);
%%
figure(2)
subplot(3,1,1)
hold on;grid on;
plot(tLog,deltaULog(1:3,:))
title("Torques commanded", 'FontSize', 15, Interpreter="latex")
legend("$M_{x}$ ", "$M_{y}$", "$M_{z}$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Forces (N)')


subplot(3,1,2)
grid on; hold on;
plot(tLog,thrustPushLog(1:3,:))
title("Thrusters Torques", 'FontSize', 15, Interpreter="latex")
legend("$M_x$ ", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])

subplot(3,1,3)
grid on; hold on;
plot(tLog,reactionWheelTorqueLog)
title("Wheel torques", 'FontSize', 15, Interpreter="latex")
legend("$M_x$", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])


%%
figure;hold on;grid on;
plot(tLog,reactionWheelSpeedLog)
title("Wheel Speed", 'FontSize', 15, Interpreter="latex")
legend("$\omega_{1}$ ", "$\omega_{2}$ ","$\omega_{3}$ ","$\omega_{4}$ ",Interpreter="latex")
xlabel('Time (s)')
ylabel('\omega (rad/s)')
%%
figure;
subplot(2,1,1)
grid on; hold on;
plot(tLog,thrustInputLog(1:3,:))
title("Thrusters Torques Input", 'FontSize', 15, Interpreter="latex")
legend("$M_x$ ", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])
subplot(2,1,2)
grid on; hold on;
plot(tLog,thrustPushLog(1:3,:))
title("Thrusters Torques", 'FontSize', 15, Interpreter="latex")
legend("$M_x$ ", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])