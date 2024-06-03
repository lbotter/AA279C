function reactionWheelTorque = reactionWheelActuator(torqueCmd)
    persistent motorCmd motorCmdPrev motorAct motorActPrev
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
    motorAct = min(1, max(-1, motorAct)); % saturate @ 1 Nm for each of the 4 wheels
    reactionWheelTorque = actuatorMatrix*motorAct;

    motorActPrev = motorAct;
    motorCmdPrev = motorCmd;
end