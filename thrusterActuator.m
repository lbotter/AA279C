function [uOutput,T] = thrusterActuator(uInput,tmin,tmax,A,E)
%THRUSTERACTUATOR function to use thrusters
%   INPUT:      uInput  :   desired thrust from flight sfw  [6x1]
%               tmin    :   minimum thrust                  [N]
%               tmax    :   maximum thrust                  [N]
%               A       :   thrust matrix                   [nx3]
%               fuelIn  :   initial fuel                    [kg]  
%               E       :   thrusters mounting directions   [3xn]
%   OUTPUT:     uOutput :   resultin torque/force to sys    [6x1]
%               fuelFina:   remaining fuel
AStar=pinv(A);
EStar=pinv(E);
uOutput=uInput;
T=AStar*uInput(1:3)+EStar*uInput(4:6);
k=0;
if max(T)>=tmax
    k=-tmax+max(T);
end
if min(T)<=tmin
    k=-min(T)+tmin;
end
T=T+k*ones(length(T),1);
uOutput(1:3)=A*T;
uOutput(4:6)=E*T;
dU=uOutput-uInput;
end

