function [reactionWheelTorque, wsOut] = reactionWheelActuator(x, torqueCmd, rWheel, dt)
    persistent motorCmd motorCmdPrev motorAct motorActPrev wheelSpeeds
    if isempty(wheelSpeeds)
        wheelSpeeds = [0;0;0;0];
    end

    [wx, wy, wz] = deal(x(5), x(6), x(7));
    
    Lw = [rWheel.Iz*wheelSpeeds(1);
          rWheel.Iz*wheelSpeeds(2);
          rWheel.Iz*wheelSpeeds(3);
          rWheel.Iz*wheelSpeeds(4)];

    actuatorMatrix = 1/sqrt(3) * [-1  1  1 -1;
                                  -1 -1  1  1;
                                   1  1  1  1];

    actuatorMatrixInv = 3/(4*sqrt(3)) * [-1 -1  1;
                                          1 -1  1;
                                          1  1  1;
                                         -1  1  1];

    motorCmd = actuatorMatrixInv * torqueCmd + normrnd(0, 0.01, 4, 1);

    if isempty(motorActPrev)
        motorActPrev = [0;0;0;0];
        motorCmdPrev = motorCmd;
    end

    motorAct = 0.90483741803596*motorActPrev + 0.0951625819640404*motorCmdPrev; % 1/(tau + s) delay tau = 0.03 s
    motorAct = min(rWheel.torqueLim, max(-rWheel.torqueLim, motorAct)); % saturate @ 1 Nm for each of the 4 wheels
    reactionWheelTorque = actuatorMatrix*motorAct + cross([wx; wy; wz], actuatorMatrix*Lw);

    alphaW = (1/rWheel.Iz)*motorAct;
    wheelSpeeds = wheelSpeeds + dt*alphaW;
    wsOut = wheelSpeeds;

    motorActPrev = motorAct;
    motorCmdPrev = motorCmd;
end