function [t,y] = Euler_wheel(I,w0,r,Iw,t_final,M)
% 
function dwdt=odefun(t,w)
    dwdt=zeros(4,1);
    dwdt(4)=0;
    dwdt(1)=M(1)-(+Iw*dwdt(4)*r(1)+(I(3,3)-I(2,2))*w(2)*w(3)+Iw*w(4)*(w(2)*r(3)-w(3)*r(2)))/I(1,1);
    dwdt(2)=M(2)-(+Iw*dwdt(4)*r(2)+(I(1,1)-I(3,3))*w(3)*w(1)+Iw*w(4)*(w(3)*r(1)-w(1)*r(3)))/I(2,2);
    dwdt(3)=M(3)-(+Iw*dwdt(4)*r(3)+(I(2,2)-I(1,1))*w(1)*w(2)+Iw*w(4)*(w(1)*r(2)-w(2)*r(1)))/I(3,3);
end
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan = [0 t_final];
[t, y] = ode113(@odefun, tspan, w0,options);
end


