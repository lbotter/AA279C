function [t,y] = Euler_ode(I,w0,t_final)
function dwdt=odefun(t,w)
    dwdt=zeros(3,1);
    dwdt(1)=-(I(3,3)-I(2,2))*w(2)*w(3)/I(1,1);
    dwdt(2)=-(I(1,1)-I(3,3))*w(1)*w(3)/I(2,2);
    dwdt(3)=-(I(2,2)-I(1,1))*w(2)*w(1)/I(3,3);
end
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
tspan = [0 t_final];
[t, y] = ode113(@odefun, tspan, w0,options);
end

