% time response of linear body velocity ROV RRC 1/2
figure 
subplot(2,3,1);
plot(t, bout(:,1));
xlabel('Time(sec)')
ylabel('Surge vel(m/s)')

subplot(2,3,2);
plot(t, bout(:,2));
title('Time response');
xlabel('Time(sec)')
ylabel('Sway vel(m/s)')


subplot(2,3,3);
plot(t, bout(:,3)); 
xlabel('Time(sec)')
ylabel('Heave vel(m/s)')


subplot(2,3,4);
plot(t, bout(:,4)); 
xlabel('Time(sec)')
ylabel('Roll rate(rad/s)')


subplot(2,3,5);
plot(t, bout(:,5)); 
xlabel('Time(sec)')
ylabel('Pitch rate(rad/s)')

subplot(2,3,6); 
plot(t, bout(:,6));
xlabel('Time(sec)')
ylabel('Yaw rate(rad/s)')

figure 
subplot(2,3,1);
plot(t, eout(:,1));
xlabel('Time(sec)')
ylabel('Surge pos(m)')

subplot(2,3,2);
plot(t, eout(:,2));
title('Time response');
xlabel('Time(sec)')
ylabel('Sway pos(m)')

subplot(2,3,3);
plot(t, eout(:,3)); 
xlabel('Time(sec)')
ylabel('Heave pos(m)')

subplot(2,3,4);
plot(t, eout(:,4)); 
xlabel('Time(sec)')
ylabel('Roll angle(rad)')

subplot(2,3,5);
plot(t, eout(:,5)); 
xlabel('Time(sec)')
ylabel('Pitch angle(rad)')

subplot(2,3,6); 
plot(t, eout(:,6));
xlabel('Time(sec)')
ylabel('Yaw angle(rad)')


