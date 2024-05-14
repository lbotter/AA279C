%% Run Simulation

for k = 0:length(0:dt:tf)
    % Propagate to the next step
    % Compute the rotation matrix
    R_P2I=principal_to_inertia(x(1:4));
    % Velocity in inertial frame from principal
    v_inertia= R_P2I*[x(11); x(12); x(13)];
     
    [R,T,N]=LVL_frame(x(8),x(9),x(10),v_inertia);
    R_P2I=inv(R_P2I);
    R=R_P2I*R;
    T=R_P2I*T;
    N=R_P2I*N;
    
    % Uncomment for gravity gradient
    u(1:3)=GravityGradient(I_principal,R,norm([x(8);x(9);x(10)]));
    xNew = fDiscreteRK4(x, u, dt);

    uLog(:,k+1) = u;
    xLog(:,k+1) = x;
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
    % x: state, [q0; q1; q2; q3; wx; wy; wz]
    % u: input, [tx; ty; tz]
    
    % Outputs:
    % xDot
    % % parameters
    % Ixx = 1.0e+03 * 0.230242103134935;
    % Iyy = 1.0e+03 * 5.293968876196306;
    % Izz = 1.0e+03 * 5.518363350668759;
    % %Edit 1
    % Ixx = 1.0e+03 * 1.230242103134935;
    % Iyy = 1.0e+03 * 1.693968876196306;
    % Izz = 1.0e+03 * 1.518363350668759;

    % Edit 2
    Ixx = 1.0e+03 * 1.930242103134935;
    Iyy = 1.0e+03 * 1.193968876196306;
    Izz = 1.0e+03 * 1.618363350668759;

    % Satellite mass
    m = 260;
    muv = 3.986004418e14;
    % Wheel parameters

    IwheelZ = 0;  % kg*m^2
    wWheel = 0; % rad/s

    % Wheel orientation
    %[rx,ry,rz]=[0.0630, -0.9979, 0.0136];
    rx=0.0630;
    ry=-0.9979;
    rz=0.0136;

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

    xDot = [-0.5*q1*wx - 0.5*q2*wy - 0.5*q3*wz;
            0.5*q0*wx - 0.5*q3*wy + 0.5*q2*wz;
            0.5*q3*wx + 0.5*q0*wy - 0.5*q1*wz;
            0.5*q1*wy - 0.5*q2*wx + 0.5*q0*wz;
           (tx - (Izz-Iyy)*wy*wz+IwheelZ*wWheel*(wy*rz-wz*ry))/Ixx;
           (ty - (Ixx-Izz)*wz*wx+IwheelZ*wWheel*(wz*rx-wx*rz))/Iyy;
           (tz - (Iyy-Ixx)*wx*wy+IwheelZ*wWheel*(wx*ry-wy*rx))/Izz;
            vx*(q0^2 + q1^2 - q2^2 - q3^2) - vy*(2*q0*q3 - 2*q1*q2) + vz*(2*q0*q2 + 2*q1*q3);
            vy*(q0^2 - q1^2 + q2^2 - q3^2) + vx*(2*q0*q3 + 2*q1*q2) - vz*(2*q0*q1 - 2*q2*q3);
            vz*(q0^2 - q1^2 - q2^2 + q3^2) - vx*(2*q0*q2 - 2*q1*q3) + vy*(2*q0*q1 + 2*q2*q3);
           (fx + m*vy*wz - m*vz*wy - (m*muv*px*(q0^2 + q1^2 - q2^2 - q3^2))/(px^2 + py^2 + pz^2)^1.5 - (m*muv*py*(2*q0*q3 + 2*q1*q2))/(px^2 + py^2 + pz^2)^1.5 + (m*muv*pz*(2*q0*q2 - 2*q1*q3))/(px^2 + py^2 + pz^2)^1.5)/m;
           (fy - m*vx*wz + m*vz*wx - (m*muv*py*(q0^2 - q1^2 + q2^2 - q3^2))/(px^2 + py^2 + pz^2)^1.5 + (m*muv*px*(2*q0*q3 - 2*q1*q2))/(px^2 + py^2 + pz^2)^1.5 - (m*muv*pz*(2*q0*q1 + 2*q2*q3))/(px^2 + py^2 + pz^2)^1.5)/m;
           (fz + m*vx*wy - m*vy*wx - (m*muv*pz*(q0^2 - q1^2 - q2^2 + q3^2))/(px^2 + py^2 + pz^2)^1.5 - (m*muv*px*(2*q0*q2 + 2*q1*q3))/(px^2 + py^2 + pz^2)^1.5 + (m*muv*py*(2*q0*q1 - 2*q2*q3))/(px^2 + py^2 + pz^2)^1.5)/m];

end