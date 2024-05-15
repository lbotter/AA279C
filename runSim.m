%% Run Simulation

% Main loop
for k = 0:length(0:dt:tf)
    currentTime = tLog(k+1);
    R_P2I = principal2Inertial(x(1:4));
    % Velocity in inertial frame from principal
    vInertial = R_P2I*[x(11); x(12); x(13)];
    [R,T,N] = inertial2RTN(x(8),x(9),x(10),vInertial);
    % ASK albert
    R_P2I = inv(R_P2I);
    R = R_P2I*R;
    T = R_P2I*T;
    N = R_P2I*N;

    [V,M,measW]=generateMeasurementMatrix(x,currentTime,0);

    % Determine the desired attitude
    qDesired=desiredAttitude(pointingPrincipal,x(8:10)',x(1:4)');

    % u: input, [tx; ty; tz; fx; fy; fz]
    gravityGradientTorque = gravityGradient(I_principal, R, norm([x(8); x(9); x(10)]));
    magneticTorque = magneticTorques(x, currentTime);
    solarTorque = solarTorques(x, geometryPrincipalFrame);
    aeroTorque = aeroTorques(x, geometryPrincipalFrame);

    % Attitude estimation
    [attitudeErr,~]=attitudeError(qDesired,x(1:4));
    attitudeEstimate=deterministicAttitude(M,V);
    %attitudeEstimateq=qmethod(M,V,measW);


    % Total torque acting on the satellite
    u(1:3) = gravityGradientTorque + magneticTorque + solarTorque + aeroTorque;


    % Update the log
    gravityTorqueLog(:,k+1) = norm(gravityGradientTorque);
    magneticTorqueLog(:,k+1) = norm(magneticTorque);
    solarTorqueLog(:,k+1) = norm(solarTorque);
    aeroTorqueLog(:,k+1) = norm(aeroTorque);

    % Onboard Estimate
    attitudeErrorLog(:,k+1) =attitudeErr;

    % attitudeEstimateLog(:,k+1) = norm(attitudeEstimate);

    % GroundTruth update
    uLog(:,k+1) = u;
    xLog(:,k+1) = x;
    xNew = fDiscreteRK4(x, u, dt);
    x = xNew;

end


%% functions

% Orbit time propagator with RK4 algorithm


function xNew = fDiscreteRK4(x, u, dt)
    % Performs a forward RK4 step assuming a zero-order-held u
    k1 = fContinuous(x, u);
    k2 = fContinuous(x + dt*k1/2, u);
    k3 = fContinuous(x + dt*k2/2, u);
    k4 = fContinuous(x + dt*k3, u);
    
    xNew = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    xNew(1:4) = xNew(1:4)./norm(xNew(1:4)); % normalize quaternion
end

%%

function xDot = fContinuous(x, u)
    % Returns the nonlinear continuous-time derivative, x_dot = f(x, u)
    
    % Inputs:
    % x: state, [q0; q1; q2; q3; wx; wy; wz; px; py; pz; vx; vy; vz]
    % u: input, [tx; ty; tz; fx; fy; fz]
    
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
    % Reaction Wheel
    IwheelZ=0;
    wWheel=0;
    % Direction of the Wheel principal axis
    rx=0;
    ry=0;
    rz=0;

    xDot = [-0.5000*q1*wx - 0.5000*q2*wy - 0.5000*q3*wz;
             0.5000*q0*wx - 0.5000*q3*wy + 0.5000*q2*wz;
             0.5000*q3*wx + 0.5000*q0*wy - 0.5000*q1*wz;
             0.5000*q1*wy - 0.5000*q2*wx + 0.5000*q0*wz;
           (tx - (Izz-Iyy)*wy*wz+IwheelZ*wWheel*(wy*rz-wz*ry))/Ixx;
           (ty - (Ixx-Izz)*wz*wx+IwheelZ*wWheel*(wz*rx-wx*rz))/Iyy;
           (tz - (Iyy-Ixx)*wx*wy+IwheelZ*wWheel*(wx*ry-wy*rx))/Izz;
             vx*(q0^2 + q1^2 - q2^2 - q3^2) - vy*(2*q0*q3 - 2*q1*q2) + vz*(2*q0*q2 + 2*q1*q3);
             vy*(q0^2 - q1^2 + q2^2 - q3^2) + vx*(2*q0*q3 + 2*q1*q2) - vz*(2*q0*q1 - 2*q2*q3);
             vz*(q0^2 - q1^2 - q2^2 + q3^2) - vx*(2*q0*q2 - 2*q1*q3) + vy*(2*q0*q1 + 2*q2*q3);
            (fx + m*vy*wz - m*vz*wy - (m*mu*px*(q0^2 + q1^2 - q2^2 - q3^2))/(px^2 + py^2 + pz^2)^1.5 - (m*mu*py*(2*q0*q3 + 2*q1*q2))/(px^2 + py^2 + pz^2)^1.5 + (m*mu*pz*(2*q0*q2 - 2*q1*q3))/(px^2 + py^2 + pz^2)^1.5)/m;
            (fy - m*vx*wz + m*vz*wx - (m*mu*py*(q0^2 - q1^2 + q2^2 - q3^2))/(px^2 + py^2 + pz^2)^1.5 + (m*mu*px*(2*q0*q3 - 2*q1*q2))/(px^2 + py^2 + pz^2)^1.5 - (m*mu*pz*(2*q0*q1 + 2*q2*q3))/(px^2 + py^2 + pz^2)^1.5)/m;
            (fz + m*vx*wy - m*vy*wx - (m*mu*pz*(q0^2 - q1^2 - q2^2 + q3^2))/(px^2 + py^2 + pz^2)^1.5 - (m*mu*px*(2*q0*q2 + 2*q1*q3))/(px^2 + py^2 + pz^2)^1.5 + (m*mu*py*(2*q0*q1 - 2*q2*q3))/(px^2 + py^2 + pz^2)^1.5)/m];

end