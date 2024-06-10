% SCRIPT TO PLOT ALL THE QUANTITIES RELATED TO THE DESATURATION MODE

%% Thrusters
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

%% Torques division
figure;
subplot(3,1,1)
hold on;grid on;
plot(tLog,deltaULog(1:3,:))
title("Control Torques", 'FontSize', 15, Interpreter="latex")
legend("$M_{x}$ ", "$M_{y}$", "$M_{z}$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Forces (N)')


subplot(3,1,2)
grid on; hold on;
plot(tLog,thrustPushLog(1:3,:))
title("Thrusters Torque", 'FontSize', 15, Interpreter="latex")
legend("$M_x$ ", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])

subplot(3,1,3)
grid on; hold on;
plot(tLog,reactionWheelTorqueLog)
title("Wheels Torque", 'FontSize', 15, Interpreter="latex")
legend("$M_x$", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])


%% Wheels Speed
figure;hold on;grid on;
plot(tLog,reactionWheelSpeedLog)
title("Wheels Speed", 'FontSize', 15, Interpreter="latex")
legend("$\omega_{1}$ ", "$\omega_{2}$ ","$\omega_{3}$ ","$\omega_{4}$ ",Interpreter="latex")
xlabel('Time (s)')
ylabel('\omega (rad/s)')
%% Thrusters input vs real
figure;
subplot(2,1,1)
grid on; hold on;
plot(tLog,thrustLog)
title("Thrusters Force", 'FontSize', 15, Interpreter="latex")
legend("$t_1$ ", "$t_2$ ","$t_3$ ","$t_4$ ",Interpreter="latex")
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0,tLog(end)])
subplot(2,1,2)
grid on; hold on;
plot(tLog,thrustPushLog(1:3,:))
title("Thrusters Torques", 'FontSize', 15, Interpreter="latex")
legend("$M_x$ ", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])