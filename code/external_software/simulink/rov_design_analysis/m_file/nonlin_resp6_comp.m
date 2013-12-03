% time response of linear body velocity ROV RRC 1/2
figure(1)
subplot(2,3,1);
plot(t, bout(:,1),'-',t,bout1(:,1),'--')
xlabel('Time(sec)')
ylabel('Surge vel(m/s)')

subplot(2,3,2);
plot(t, bout(:,2),'-',t,bout1(:,2),'--')
title('Time response');
xlabel('Time(sec)')
ylabel('Sway vel(m/s)')
legend('w perturb','w/o perturb')


subplot(2,3,3);
plot(t, bout(:,3),'-',t,bout1(:,3),'--')
xlabel('Time(sec)')
ylabel('Heave vel(m/s)')


subplot(2,3,4);
plot(t, bout(:,4),'-',t,bout1(:,4),'--')
xlabel('Time(sec)')
ylabel('Roll rate(rad/s)')


subplot(2,3,5);
plot(t, bout(:,5),'-',t,bout1(:,5),'--') 
xlabel('Time(sec)')
ylabel('Pitch rate(rad/s)')

subplot(2,3,6); 
plot(t, bout(:,6),'-',t,bout1(:,6),'--')
xlabel('Time(sec)')
ylabel('Yaw rate(rad/s)')

figure(2)
subplot(2,3,1);
plot(t, eout(:,1),'-',t,eout1(:,1),'--')
xlabel('Time(sec)')
ylabel('Surge pos(m)')

subplot(2,3,2);
plot(t, eout(:,2),'-',t,eout1(:,2),'--')
title('Time response');
xlabel('Time(sec)')
ylabel('Sway pos(m)')

subplot(2,3,3);
plot(t, eout(:,3),'-',t,eout1(:,3),'--')
xlabel('Time(sec)')
ylabel('Heave pos(m)')
legend('w perturb','w/o perturb')

subplot(2,3,4);
plot(t, eout(:,4),'-',t,eout1(:,4),'--')
xlabel('Time(sec)')
ylabel('Roll angle(rad)')

subplot(2,3,5);
plot(t, eout(:,5),'-',t,eout1(:,5),'--')
xlabel('Time(sec)')
ylabel('Pitch angle(rad)')

subplot(2,3,6); 
plot(t, eout(:,6),'-',t,eout1(:,6),'--')
xlabel('Time(sec)')
ylabel('Yaw angle(rad)')


