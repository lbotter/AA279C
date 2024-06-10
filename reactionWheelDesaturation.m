function [uThruster,t] = reactionWheelDesaturation(rWheel,wheelSpeeds,dt,t)
%REACTIONWHEELDESATURATION: Function to reduce wheel speed
%   INPUT   :   rWheel      :   reaction wheel parameters
%           :   weelSpeeds  :   reaction wheel speeds   
%           :   t           :   time left to finish the maneuver
%           :   dt          :   time step
%   OUTPUT  :   wWheel      :   wheel speeds
%           :   uThruster   :  thrusters torques

persistent LwDot

if t==0
    t=floor(max(wheelSpeeds)/rWheel.dw);
    dw = wheelSpeeds/t;
LwDot = [rWheel.Iz*dw(1)/dt;
          rWheel.Iz*dw(2)/dt;
          rWheel.Iz*dw(3)/dt;
          rWheel.Iz*dw(4)/dt];
end
uThruster=rWheel.matrix*LwDot;
uThruster=[uThruster;0;0;0];
t=t-1;
if t<0
    t=0;
end

end