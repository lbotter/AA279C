Params;
span=100;
thrustLog       = zeros(4,(span)+2);
thrustPushLog      = zeros(6,(span)+2);
tLog=zeros(span,1);
thrustLog=zeros(4,span);
thrustPushLog=zeros(6,span);
for i=1:100
    uReq=[0;0;1;0;0;0];
    if i<80
    [uThrusters,T]=thrusterActuator(uReq,Thruster,dt);
    else 
        [uThrusters,T]=thrusterActuator([0;0;0;0;0;0],Thruster,dt);
    end
    thrustLog(:,i)=T;
    thrustPushLog(:,i)=uThrusters;
    tLog(i)=i;
end
%% Plot

clear ylim
subplot(3,1,1)
grid on; hold on;
plot(tLog,thrustPushLog(1:3,:))
title("Torques", 'FontSize', 15, Interpreter="latex")
legend("$M_x$ ", "$M_y$", "$M_z$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Torque (Nm)')
xlim([0,tLog(end)])
ylim([0 1.2])


subplot(3,1,2)
grid on; hold on;
plot(tLog,thrustPushLog(4:6,:))
title("Forces", 'FontSize', 15, Interpreter="latex")
legend("$F_{x}$ ", "$F_{y}$", "$F_{z}$",Interpreter="latex")
xlabel('Time (s)')
ylabel('Forces (N)')
ylim([0 1.2])

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
ylim([0 1.1])