% Nonlinear analysis- energy/passivity
clc;

disp('V > 0')

norm_V=sqrt(sum(bout1.^2))*M*sqrt(sum(bout1.^2))'
norm_V_vert_hort=sqrt(sum(bout.^2))*M*sqrt(sum(bout.^2))'

disp('Vdot=v^T*\tau-v^TDv')
Vright2= sqrt(sum(bout1.^2))*sqrt(sum(u1.^2))'-sqrt(sum(bout1.^2))*-(DQL+DQ)*sqrt(sum(bout1.^2))'
Vright_vert_hort= sqrt(sum(bout.^2))*sqrt(sum(u.^2))'-sqrt(sum(bout.^2))*-(DQL+DQ)*sqrt(sum(bout.^2))'

norm_Vdot=sqrt(sum(Vright2'.^2))
norm_Vdot_vert_hort=sqrt(sum(Vright_vert_hort'.^2))


%-------------Full-order-----------------
figure
subplot(3,2,1)
plot(t,V(:,1))
xlabel('Time(sec)')
ylabel('V_x')
title('Energy- Full-Order')

subplot(3,2,2)
plot(t,V(:,2))
xlabel('Time(sec)')
ylabel('V_y')

subplot(3,2,3)
plot(t,V(:,3))
xlabel('Time(sec)')
ylabel('V_z')

subplot(3,2,4)
plot(t,V(:,4))
xlabel('Time(sec)')
ylabel('V_{\phi}')

subplot(3,2,5)
plot(t,V(:,5))
xlabel('Time(sec)')
ylabel('V_{\theta}')

subplot(3,2,6)
plot(t,V(:,6))
xlabel('Time(sec)')
ylabel('V_{\psi}')


%-------------station-keeping-----------------
figure(7)
subplot(4,1,1)
plot(t,V(:,1))
xlabel('Time(sec)')
ylabel('V_x')
title('Energy- Station-keeping')

subplot(4,1,2)
plot(t,V(:,2))
xlabel('Time(sec)')
ylabel('V_y')

subplot(4,1,3)
plot(t,V(:,3))
xlabel('Time(sec)')
ylabel('V_z')

subplot(4,1,4)
plot(t,V(:,6))
xlabel('Time(sec)')
ylabel('V_{\psi}')

 
figure(8)
subplot(4,1,1)
plot(t,Vdot(:,1),'-',t,Vright(:,1),'--')
xlabel('Time(sec)')
ylabel('x')
title('Vdot - Station-keeping')
legend('Vdot','Vright')

subplot(4,1,2)
plot(t,Vdot(:,2),'-',t,Vright(:,2),'--')
xlabel('Time(sec)')
ylabel('y')
legend('Vdot','Vright')

subplot(4,1,3)
plot(t,Vdot(:,3),'-',t,Vright(:,3),'--')
xlabel('Time(sec)')
ylabel('z')
legend('Vdot','Vright')

subplot(4,1,4)
plot(t,Vdot(:,6),'-',t,Vright(:,6),'--')
xlabel('Time(sec)')
ylabel('\psi')
legend('Vdot','Vright')


%-----------------------full- order-----------------------------
figure 
subplot(3,2,1)
plot(t,Vdot(:,1),'-',t,Vright(:,1),'--')
xlabel('Time(sec)')
ylabel('x')
title('Vdot - Full Order')
legend('Vdot','Vright')

subplot(3,2,2)
plot(t,Vdot(:,2),'-',t,Vright(:,2),'--')
xlabel('Time(sec)')
ylabel('y')
legend('Vdot','Vright')

subplot(3,2,3)
plot(t,Vdot(:,3),'-',t,Vright(:,3),'--')
xlabel('Time(sec)')
ylabel('z')
legend('Vdot','Vright')

subplot(3,2,4)
plot(t,Vdot(:,4),'-',t,Vright(:,4),'--')
xlabel('Time(sec)')
ylabel('\phi')
legend('Vdot','Vright')

subplot(3,2,5)
plot(t,Vdot(:,5),'-',t,Vright(:,5),'--')
xlabel('Time(sec)')
ylabel('\theta')
legend('Vdot','Vright')

subplot(3,2,6)
plot(t,Vdot(:,6),'-',t,Vright(:,6),'--')
xlabel('Time(sec)')
ylabel('\psi')
legend('Vdot','Vright')

%--------------Hort-----------------------------------

figure(9)
subplot(3,1,1)
plot(t,V(:,1))
xlabel('Time(sec)')
ylabel('V_x')
title('Energy - Horizontal Subsystem ')

subplot(3,1,2)
plot(t,V(:,2))
xlabel('Time(sec)')
ylabel('V_y')

subplot(3,1,3)
plot(t,V(:,6))
xlabel('Time(sec)')
ylabel('V_{\psi}')

 
figure(10)
subplot(3,1,1)
plot(t,Vdot(:,1),'-',t,Vright(:,1),'--')
xlabel('Time(sec)')
ylabel('x')
title('Vdot - Horizontal Subsystem')
legend('Vdot','Vright')

subplot(3,1,2)
plot(t,Vdot(:,2),'-',t,Vright(:,2),'--')
xlabel('Time(sec)')
ylabel('y')
legend('Vdot','Vright')

subplot(3,1,3)
plot(t,Vdot(:,6),'-',t,Vright(:,6),'--')
xlabel('Time(sec)')
ylabel('\psi')
legend('Vdot','Vright')



%--------------Vert-----------------------------------

figure(11)
subplot(3,1,1)
plot(t,V(:,1))
xlabel('Time(sec)')
ylabel('V_x')
title('Energy - Vertical Subsystem')

subplot(3,1,2)
plot(t,V(:,3))
xlabel('Time(sec)')
ylabel('V_z')

subplot(3,1,3)
plot(t,V(:,5))
xlabel('Time(sec)')
ylabel('V_{\theta}')

 
figure(12)
subplot(3,1,1)
plot(t,Vdot(:,1),'-',t,Vright(:,1),'--')
xlabel('Time(sec)')
ylabel('x')
title(' Vdot - Vertical Subsystem')
legend('Vdot','Vright')

subplot(3,1,2)
plot(t,Vdot(:,3),'-',t,Vright(:,3),'--')
xlabel('Time(sec)')
ylabel('z')
legend('Vdot','Vright')

subplot(3,1,3)
plot(t,Vdot(:,5),'-',t,Vright(:,5),'--')
xlabel('Time(sec)')
ylabel('\theta')
legend('Vdot','Vright')

