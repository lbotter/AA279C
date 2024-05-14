% Script to correct plot for PS3

Params;

t_final=120;
[t,w]=Euler_ode(I_principal,w0,t_final);

I=I_principal;
L=I_principal*w0;
theta= acos(L(3)/norm(L));
gamma=acos(w0(3)/norm(w0));
lambda=(I(3,3)-I(1,1))/I(1,1)*w0(3);
wxy=@(t) (w0(1)+1i*w0(2))*exp(1i*lambda*t);

time=t;
w_analytical=zeros(3,length(time));
w_analytical(1,:)=real(wxy(time));
w_analytical(2,:)=imag(wxy(time));
w_analytical(3,:) = ones(length(time),1)*w0(3);

figure(2)
plot(t,w,'-')
xlabel('t (s)');
ylabel('\omega (rad/s)');
title('Euler integration');
legend('\omega_x','\omega_y','\omega_z')
hold off
