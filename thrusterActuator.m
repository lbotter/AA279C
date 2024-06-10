function [uOutput,T] = thrusterActuator(uInput,Tr,dt)
%THRUSTERACTUATOR function to use thrusters
%   INPUT:      uInput  :   desired thrust from flight sfw  [6x1]
%               Thruster:   Thruster object
%   OUTPUT:     uOutput :   resultin torque/force to sys    [6x1]
%               fuelFina:   remaining fuel
persistent TCmd TCmdPrev TAct TActPrev
AStar=pinv(Tr.A);
EStar=pinv(Tr.E);
tmax=Tr.Tmax;
tmin=Tr.Tmin;
uOutput=uInput;
TCmd=AStar*uInput(1:3)+EStar*uInput(4:6);
k=0;
if max(TCmd)>=tmax
    k=-tmax+max(TCmd);
end
TCmd=TCmd+k*ones(length(TCmd),1);
if min(TCmd)<=tmin
    k=-min(TCmd)+tmin;
end
TCmd=TCmd+k*ones(length(TCmd),1);
if isempty(TActPrev)
    TActPrev = [0;0;0;0];
    TCmdPrev = TCmd;
end

tau = 0.01;  % Time constant for delay
delta_t = dt;
alpha = delta_t / (tau + delta_t);

if norm(uInput)<1e-6
    TCmd=0*TCmd;
end
TAct = (1 - alpha) * TActPrev + alpha * TCmdPrev;  % Updated delay equation
TActPrev=TAct;
uOutput(1:3)=Tr.A*TAct;
uOutput(4:6)=Tr.E*TAct;
T=TAct;
TCmdPrev = TCmd;

end

