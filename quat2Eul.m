function eulerPhiThetaPsi = quat2Eul(quat)

q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

eulerPhiThetaPsi = [atan2(2*(q0*q1 + q2*q3), 1-2*(q1^2 + q2^2));
                   -3.14159265/2 + 2*atan2(sqrt(1 + 2*(q0*q2 - q1*q3)), sqrt(1 - 2*(q0*q2 - q1*q3)));
                    atan2(2*(q0*q3 + q1*q2), 1-2*(q2^2 + q3^2))];

end